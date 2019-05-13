#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Header.h>
#include <queue>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <cstring>
#include <string>

#include <loguru.hpp>

#include "../estimation/lidar_factor.hpp"

#include "../estimation/state_parameterization.hpp"
#include "../estimation/wheel_factor.hpp"
#include "../estimation/wheel_integrator.hpp"
#include "../utility/utility.h"

int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

using namespace loam;
//wheel
std::condition_variable con;
std::mutex m_buf; // process conflict of imu_buf and laser
std::mutex m_state; // process conflict of multiOdo (tmp_P、tmp_Q、tmp_V)
bool init_wheel = 1;
double last_wheel_t = 0;

Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q(1, 0, 0, 0);
Eigen::Vector3d tmp_V;
std::vector<std::pair<double, Eigen::Vector3d>> velVector;
std::vector<std::pair<double, Eigen::Vector3d>> gyrVector;
std::vector<RawWheel> wheels;
double latest_time = 0;
Eigen::Vector3d latest_P, latest_V, latest_vel_0, latest_gyr_0;
Eigen::Quaterniond latest_Q;
ros::Publisher pubWheelOdometry;
ros::Publisher pubWheelPath;
nav_msgs::Path wheelPath;
bool USE_WHEEL = true;
bool pub_wheel;
bool saveLaserOdoINI = true;
bool saveWheelOdo = true;
bool initFirstPoseFlag;

double prevTime = 0;
double curTime = 0;
double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;
double timeWheel = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;


Eigen::Vector3d vel_0, gyr_0;
// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

loam::State state_i;
loam::State state_j;

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::queue<std::pair<double, Eigen::Vector3d>> velBuf;
std::queue<std::pair<double, Eigen::Vector3d>> gyrBuf;
std::mutex mBuf;

bool WheelAvailable(double t)
{
    if(!velBuf.empty() && t <= velBuf.back().first)
        return true;
    else
        return false;
}

void inputWheel(double t, const Eigen::Vector3d &linearVelocity, const Eigen::Vector3d &angularVelocity)
{
    mBuf.lock();
    velBuf.push(std::make_pair(t, linearVelocity));
    gyrBuf.push(std::make_pair(t, angularVelocity));
    mBuf.unlock();
    //con.notify_one();
}

bool getWheelInterval(double t0, double t1, std::vector<std::pair<double, Eigen::Vector3d>> &velVector,
                      std::vector<std::pair<double, Eigen::Vector3d>> &gyrVector){
    if (velBuf.empty()){
        LOG_F(INFO, "not receive wheel\n");
        return false;
    }

    if(t1 <= velBuf.back().first)
    {
        while (velBuf.front().first <= t0)
        {
            velBuf.pop();
            gyrBuf.pop();
        }
        while (velBuf.front().first < t1)
        {
            velVector.push_back(velBuf.front());
            velBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        velVector.push_back(velBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        LOG_F(INFO, "wait for wheel\n");
        return false;
    }
    return true;
}

void initFirstWheelPose(double t0, double t1, std::vector<std::pair<double, Eigen::Vector3d>> &gryVector)
{
    LOG_F(INFO, "init first wheel pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averGry(0, 0, 0);  //hree is velocity
    int n = (int)gryVector.size();
    for(size_t i = 0; i < gryVector.size(); i++)
    {
        averGry = averGry + gryVector[i].second;
    }
    averGry = averGry / n;

    state_i.q = Utility::deltaQ(averGry * (t1 - t0));
//    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
//    Eigen::Matrix3d R0 = Utility::g2R(averAcc);
//    double yaw = Utility::R2ypr(R0).x();
//    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
//    state_i.q.matrix() = R0;
//    Rs[0] = R0;
//    cout << "init R0 " << endl << Rs[0] << endl;
//    Vs[0] = Vector3d(5, 0, 0);
}

// undistort lidar point
void TransformToStart(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    //interpolation ratio
    double s = 1.0;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}

void wheelHandler(const geometry_msgs::TwistStampedConstPtr &wheel_msg)
{
    using Eigen::Vector3d;

    if (wheel_msg->header.stamp.toSec() <= last_wheel_t)
    {
        ROS_WARN("wheel message in disorder!");
        return;
    }

    double t  = wheel_msg->header.stamp.toSec();
    last_wheel_t = t;
    double vx = wheel_msg->twist.linear.x;
    double vy = wheel_msg->twist.linear.y;
    double vz = wheel_msg->twist.linear.z;
    double rx = wheel_msg->twist.angular.x;
    double ry = wheel_msg->twist.angular.y;
    double rz = wheel_msg->twist.angular.z;
    Vector3d vel(vx, vy, vz);
    Vector3d gyr(rx, ry, rz);
    inputWheel(t, vel, gyr);

    if (init_wheel)
    {
        latest_time = t;
        init_wheel = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    tmp_Q = tmp_Q * Utility::deltaQ(gyr * dt);
    tmp_P = tmp_P + tmp_Q.toRotationMatrix() * vel * dt;
    tmp_V = vel;

    nav_msgs::Odometry wheelOdometry;
    wheelOdometry.header.frame_id = "/camera_init";
    wheelOdometry.child_frame_id = "/laser_odom";
    wheelOdometry.header.stamp = ros::Time().fromSec(t);
    wheelOdometry.pose.pose.orientation.x = tmp_Q.x();
    wheelOdometry.pose.pose.orientation.y = tmp_Q.y();
    wheelOdometry.pose.pose.orientation.z = tmp_Q.z();
    wheelOdometry.pose.pose.orientation.w = tmp_Q.w();
    wheelOdometry.pose.pose.position.x = tmp_P.x();
    wheelOdometry.pose.pose.position.y = tmp_P.y();
    wheelOdometry.pose.pose.position.z = tmp_P.z();
    pubWheelOdometry.publish(wheelOdometry);

    geometry_msgs::PoseStamped wheelPose;
    wheelPose.header = wheelOdometry.header;
    wheelPose.pose = wheelOdometry.pose.pose;
    wheelPath.header.stamp = wheelOdometry.header.stamp;
    wheelPath.poses.push_back(wheelPose);
    wheelPath.header.frame_id = "/camera_init";
    pubWheelPath.publish(wheelPath);

    if (saveWheelOdo) {
        std::ofstream founW("/home/zhouchang/catkin_zc/src/fusion_localization/results/wheel_odo.txt",
                            std::ios::app);
        founW.setf(std::ios::fixed, std::ios::floatfield);
        founW.precision(5);
        founW << wheelOdometry.header.stamp.toSec() << " ";
        founW.precision(5);
        founW << wheelOdometry.pose.pose.position.x << " "
              << wheelOdometry.pose.pose.position.y << " "
              << wheelOdometry.pose.pose.position.z << " "
              << wheelOdometry.pose.pose.orientation.x << " "
              << wheelOdometry.pose.pose.orientation.y << " "
              << wheelOdometry.pose.pose.orientation.z << " "
              << wheelOdometry.pose.pose.orientation.w << std::endl;
        founW.close();
    }
    return;
}

namespace detail {

    class LoggingCallback : public ceres::IterationCallback {
    public:
        virtual ~LoggingCallback() override = default;
        virtual ceres::CallbackReturnType operator()(
            const ceres::IterationSummary& summary) override {
            if (summary.iteration == 0) {
                LOG_F(INFO, "[CERES  0] fn=%16.10e", summary.cost);
                fn_old_ = summary.cost;
                tr_radius_old_ = summary.trust_region_radius;
            } else {
                LOG_F(INFO,
                      "[CERES%3d] fn=%16.10e >>>> %-16.10e "
                      "fn+=%17.10e "
                      "|step|=%16.10e "
                      "step_quality=%17.10e "
                      "tr_radius=%16.10e",
                      summary.iteration,
                      fn_old_, summary.cost,
                      -summary.cost_change,
                      summary.step_norm,
                      summary.relative_decrease,
                      tr_radius_old_);
                fn_old_ = summary.cost;
                tr_radius_old_ = summary.trust_region_radius;
            }
            return ceres::SOLVER_CONTINUE;
        }

    private:
        double fn_old_ = 0.0;
        double tr_radius_old_ = 0.0;
    };

}  /* namespace detail */

int main(int argc, char **argv)
{
    loguru::init(argc, argv);
    loguru::g_stderr_verbosity = loguru::Verbosity_MAX;
    loguru::g_preamble_thread = false;
    loguru::g_preamble_uptime = false;
    loguru::g_preamble_date = false;
    loguru::g_preamble_time = false;
    loguru::add_file("odometry.log", loguru::Truncate, loguru::Verbosity_MAX);

    ros::init(argc, argv, "LiDAROdometry");
    ros::NodeHandle nh("~");

    nh.getParam("pub_wheel", pub_wheel);
    nh.getParam("saveLaserOdoINI", saveLaserOdoINI);
    nh.getParam("saveWheelOdo", saveWheelOdo);
    nh.getParam("USE_WHEEL", USE_WHEEL);

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);

    LOG_F(INFO, "Mapping %d Hz", 10 / skipFrameNum);

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);
    ros::Subscriber subWheelIntegrator = nh.subscribe<geometry_msgs::TwistStamped>("/wheel", 200, wheelHandler, ros::TransportHints().tcpNoDelay());

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
    pubWheelOdometry = nh.advertise<nav_msgs::Odometry>("/wheel_odom_to_init", 100);
    pubWheelPath = nh.advertise<nav_msgs::Path>("/wheel_odom_path", 100);

    nav_msgs::Path laserPath;

    int frameCount = 0;
    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        velVector.clear();
        gyrVector.clear();
        wheels.clear();

        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty() && !velBuf.empty())
        {
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
            timeWheel = velBuf.front().first;
            curTime = timeSurfPointsFlat;
            LOG_F(INFO, "first_timeWheel = %11.5f"
                        "first_curTime = %11.5f"
                        "first_prevTime = %11.5f",
                  timeWheel,curTime,prevTime);

            while(USE_WHEEL)
            {
                if ((!USE_WHEEL  || WheelAvailable(curTime)))
                    break;
                else
                {
                    printf("wait for wheel data ... \n");
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
                LOG_F(INFO, "USE_WHEEL AND AVAILABLE");
            }

            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                LOG_F(INFO, "unsync messeage!");
                ROS_BREAK();
            }

            mBuf.lock();
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();

//            if (USE_WHEEL){
//                getWheelInterval(prevTime, curTime, velVector, gyrVector);
//            }
            mBuf.unlock();



            // initializing laser
            if (!systemInited)
            {
                systemInited = true;
//                state_i.arr = {1, 0, 0, 0,
//                               0, 0, 0,
//                               0, 0, 0};
                LOG_F(INFO, "Initialization finished");
            }
            else
            {
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
                {
                    corner_correspondence = 0;
                    plane_correspondence = 0;

                    ceres::Problem problem;
                    problem.AddParameterBlock(state_i.arr.data(), 10, new loam::StateParameterization);
                    problem.AddParameterBlock(state_j.arr.data(), 10, new loam::StateParameterization);

                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

                    // find correspondence for corner features
                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    {
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }
                        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                        {
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);
                            problem.AddResidualBlock(
                                new loam::PointEdgeFactor(curr_point.data(), last_point_a.data(), last_point_b.data()),
                                new ceres::HuberLoss(0.1),
                                state_i.arr.data(), state_j.arr.data());
                            corner_correspondence++;
                        }
                    }

                    // find correspondence for plane features
                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                    (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                    (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                    (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                    (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            if (minPointInd2 >= 0 && minPointInd3 >= 0)
                            {

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                           surfPointsFlat->points[i].y,
                                                           surfPointsFlat->points[i].z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                             laserCloudSurfLast->points[closestPointInd].y,
                                                             laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                             laserCloudSurfLast->points[minPointInd2].y,
                                                             laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                             laserCloudSurfLast->points[minPointInd3].y,
                                                             laserCloudSurfLast->points[minPointInd3].z);
                                problem.AddResidualBlock(
                                    new loam::PointPlanarFactor(curr_point.data(), last_point_a.data(), last_point_b.data(), last_point_c.data()),
                                    new ceres::HuberLoss(0.1),
                                    state_i.arr.data(), state_j.arr.data());
                                plane_correspondence++;
                            }
                        }
                    }

                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        LOG_F(INFO, "less correspondence! *************************************************");
                    }
                    if (USE_WHEEL){
                        mBuf.lock();
                        getWheelInterval(prevTime, curTime, velVector, gyrVector);
                        mBuf.unlock();
//                        if(!initFirstPoseFlag)
//                            initFirstWheelPose(prevTime,curTime,gyrVector);
                        for(size_t i = 0; i < velVector.size(); i++)
                        {
                            wheels.emplace_back(velVector[i].first, velVector[i].second[0], velVector[i].second[1], velVector[i].second[2],
                                                gyrVector[i].second[0], gyrVector[i].second[1], gyrVector[i].second[2]);
                        }
                        Eigen::Matrix<double, 6, 1> cov;
                        cov << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001;

                        if (wheels.size() < 2)
                            exit(0);
                        loam::WheelIntegral integral_curr = loam::WheelPreintegrator::Integrate(wheels, cov);

                        problem.AddResidualBlock(
                            new loam::WheelFactor(integral_curr),
                            new ceres::HuberLoss(0.1),
                            state_i.arr.data(), state_j.arr.data());
                    }

                    problem.SetParameterBlockConstant(state_i.arr.data());

                    detail::LoggingCallback callback;
                    ceres::Solver::Options options;
                    options.callbacks.push_back(&callback);
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.max_num_iterations = 7;
                    options.minimizer_progress_to_stdout = false;
                    ceres::Solver::Summary summary;
                    //LOG_F(INFO, "I AM HERE");
                    ceres::Solve(options, &problem, &summary);
                    LOG_S(INFO) << summary.BriefReport();
                    //LOG_F(INFO, "I AM HERE");
                    q_last_curr = state_i.q.conjugate() * state_j.q;
                    t_last_curr = state_i.q.conjugate() * (state_j.p - state_i.p);
                }
                q_w_curr = state_j.q;
                t_w_curr = state_j.p;

                //initialize state
                state_i.q = state_j.q;
                state_i.p = state_j.p;
                state_j.p = state_j.p + state_i.q * t_last_curr;
                state_j.q = state_j.q * q_last_curr;
            }

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "/camera_init";
            laserOdometry.child_frame_id = "/laser_odom";
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);

            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "/camera_init";
            pubLaserPath.publish(laserPath);

            // write result to file
            if (saveLaserOdoINI) {
                //std::ofstream founL("/home/zhouchang/catkin_fb/src/fusion_localization/results/laserWheelOdo.txt", std::ios::app);
                std::ofstream founL("/home/zhouchang/catkin_zc/src/fusion_localization/results/laserOdo.txt", std::ios::app);
                founL.setf(std::ios::fixed, std::ios::floatfield);
                founL.precision(5);
                founL << laserOdometry.header.stamp.toSec() << " ";
                founL.precision(5);
                founL << laserOdometry.pose.pose.position.x << " "
                      << laserOdometry.pose.pose.position.y << " "
                      << laserOdometry.pose.pose.position.z << " "
                      << laserOdometry.pose.pose.orientation.x << " "
                      << laserOdometry.pose.pose.orientation.y << " "
                      << laserOdometry.pose.pose.orientation.z << " "
                      << laserOdometry.pose.pose.orientation.w << std::endl;
                founL.close();
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

            if (frameCount % skipFrameNum == 0)
            {
                frameCount = 0;

                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }

            frameCount++;
        }
        prevTime = curTime;
        rate.sleep();
    }
    return 0;
}
