#include <memory>
#include <queue>
#include <utility>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>

#include "modules/msgs/localization/proto/localization.pb.h"
#include "../utility/sockvis.h"
#include <fstream>

namespace {

sockvis::udp_client udp("192.168.10.229", 12345);

std::queue<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> state_queue;
std::unique_ptr<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> x0 = nullptr;
std::unique_ptr<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> gps0 = nullptr;
double scale = 1.0;

Eigen::Matrix3d OXTSEulerToRotationMatrix(
    double roll, double pitch, double yaw) {
  double psi = yaw;
  double theta = pitch;
  double phi = roll;
  Eigen::Matrix3d R_psi, R_theta, R_phi;
  R_psi << std::cos(psi), -std::sin(psi), 0,
           std::sin(psi),  std::cos(psi), 0,
                       0,              0, 1;
  R_theta <<  std::cos(theta), 0, std::sin(theta),
                            0, 1,               0,
             -std::sin(theta), 0, std::cos(theta);
  R_phi << 1,             0,              0,
           0, std::cos(phi), -std::sin(phi),
           0, std::sin(phi),  std::cos(phi);
  return R_psi * R_theta * R_phi;
}

double LatToScale(double latitude) {
  return cos(latitude * M_PI / 180.0);
}

Eigen::Vector3d LatLonAltToMercator(
    double latitude, double longitude, double altitude, double scale) {
  double x = scale * longitude * M_PI * 6378137.0 / 180.0;
  double y = scale * 6378137.0 * log(tan((90.0 + latitude) * M_PI / 360.0));
  double z = altitude;
  return Eigen::Vector3d(x, y, z);
}

void global_callback(nav_msgs::PathConstPtr path_msg) {
  using Eigen::Quaterniond;
  using Eigen::Vector3d;

  Quaterniond q0(&path_msg->poses[0].pose.orientation.x);
  Vector3d    p0(&path_msg->poses[0].pose.position.x);

  if (not x0)
    x0 = std::make_unique<std::pair<Quaterniond, Vector3d>>(q0, p0);

  for (int i = 0; i < (int)path_msg->poses.size(); ++i) {
    const auto& pose = path_msg->poses[i];
    Quaterniond q(&pose.pose.orientation.x);
    Vector3d    p(&pose.pose.position.x);
    p = q0.conjugate() * (p - p0);
    q = q0.conjugate() * q;
    sockvis::send_pose(udp, "PUT", "global", "YELLOW", i+1, q, p);
  }
};

void odometry_callback(nav_msgs::Odometry::ConstPtr pose_msg) {
  using Eigen::Quaterniond;
  using Eigen::Vector3d;
  using Eigen::Map;

  state_queue.emplace(
      Map<const Quaterniond>(&pose_msg->pose.pose.orientation.x),
      Map<const Vector3d>(&pose_msg->pose.pose.position.x));

  if (x0) {
    while (state_queue.size()) {
      static int state_id = 0;
      const std::pair<Quaterniond, Vector3d>& x = state_queue.front();
      Vector3d p = x0->first.conjugate() * (x.second - x0->second);
      Quaterniond q = x0->first.conjugate() * x.first;
      sockvis::send_pose(udp, "PUT", "local", "RED", ++state_id, q, p);
      state_queue.pop();
    }
  }
}

void gps_callback(sensor_msgs::NavSatFixConstPtr gps_msg) {
  using Eigen::Quaterniond;
  using Eigen::Vector3d;

  if (not gps0) {
    gps0 = std::make_unique<std::pair<Quaterniond, Vector3d>>();
    scale = LatToScale(gps_msg->latitude);
    gps0->second = LatLonAltToMercator(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, scale);
  }

  Quaterniond q = Quaterniond::Identity();
  Vector3d p = LatLonAltToMercator(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, scale) - gps0->second;

  static int state_id = 0;
  sockvis::send_pose(udp, "PUT", "ground_truth", "GREEN", ++state_id, q, p);
}

void gt_callback(const roadstar::localization::Localization& gt_msg) {
    ROS_INFO("receive groundtruth msg +1.");
    using Eigen::Quaterniond;
    using Eigen::Vector3d;
    using Eigen::AngleAxisd;

    if (not gps0) {
        gps0 = std::make_unique<std::pair<Quaterniond, Vector3d>>();
        // scale = LatToScale(gps_msg->latitude);
        scale = LatToScale(gt_msg.lat());
        AngleAxisd heading_angle(gt_msg.heading(), Vector3d::UnitZ());
        AngleAxisd pitch_angle(gt_msg.pitch(), Vector3d::UnitY());
        AngleAxisd roll_angle(gt_msg.roll(), Vector3d::UnitX());
        gps0->first = heading_angle * pitch_angle * roll_angle;
        // gps0->second = LatLonAltToMercator(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, scale);
        gps0->second = LatLonAltToMercator(gt_msg.lat(), gt_msg.lon(), 0, scale);
    }

    AngleAxisd heading_angle(gt_msg.heading(), Vector3d::UnitZ());
    AngleAxisd pitch_angle(gt_msg.pitch(), Vector3d::UnitY());
    AngleAxisd roll_angle(gt_msg.roll(), Vector3d::UnitX());
    Quaterniond q_abs = heading_angle * pitch_angle * roll_angle;
    Quaterniond q = gps0->first.conjugate() * q_abs;
    // Vector3d p = LatLonAltToMercator(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, scale) - gps0->second;
    Vector3d p = gps0->first.conjugate() * (LatLonAltToMercator(gt_msg.lat(), gt_msg.lon(), 0, scale) - gps0->second);
    static int state_id = 0;
    sockvis::send_pose(udp, "PUT", "ground_truth", "GREEN", ++state_id, q, p);

    std::ofstream founL("/roadstar/modules/fusion_localization/results/gt.txt",
                        std::ios::app);
    founL.setf(std::ios::fixed, std::ios::floatfield);
    founL.precision(5);
    founL << gt_msg.header().timestamp_sec() << " ";
    founL.precision(5);
    founL << p.x() << " "
          << p.y() << " "
          << p.z() << " "
          << q.x() << " "
          << q.y() << " "
          << q.z() << " "
          << q.w() << std::endl;
    founL.close();
}


}

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_display");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  //ros::Subscriber sub_GPS  = n.subscribe("/kitti/oxts/gps/fix", 100, gps_callback);
  ros::Subscriber sub_lodm = n.subscribe("/roadstar/localization", 100, gt_callback);
  ros::Subscriber sub_path = n.subscribe("/globalEstimator/global_path", 100, global_callback);
  ros::spin();
  return 0;
}
