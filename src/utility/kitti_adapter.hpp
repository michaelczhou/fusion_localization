#ifndef LEGO_LOAM_MOD_UTILITY_KITTI_ADAPTER_HPP
#define LEGO_LOAM_MOD_UTILITY_KITTI_ADAPTER_HPP

#include <functional>
#include <queue>
#include <string>
#include <tuple>
#include <Eigen/Eigen>

namespace loam {

class KITTIAdapter {
public:
    using SensorCallbackType = std::function<void(const double*, int, long, long)>;

    KITTIAdapter(const std::string& kitti_dir, const std::string& calib_dir);

    void OnNextOXTS(SensorCallbackType&& callback);

    void OnNextVelodyne(SensorCallbackType&& callback);

    bool NextSensor();

private:
    struct TimeStamp {
        TimeStamp() = default;
        TimeStamp(long s, long ns) : second(s), nanosecond(ns)
        { }
        long second = 0;
        long nanosecond = 0;
        double seconds() const {
            return second + nanosecond * 1.0e-09;
        }
        bool operator<(const TimeStamp& other) const {
            return (second == other.second) ?
                (nanosecond < other.nanosecond) : (second < other.second);
        }
        bool operator<=(const TimeStamp& other) const {
            return not (other < (*this));
        }
    };

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d>
    GetCalibIMUToVelo(const std::string& calib_filename);

    std::queue<TimeStamp>
    LoadTimeStampNanoseconds(const std::string& timestamp_filename);

    bool NextOXTS();
    bool NextVelodyne();

    SensorCallbackType on_next_oxts_ = {};
    SensorCallbackType on_next_velodyne_ = {};

    std::string oxts_path_;
    std::string velodyne_path_;
    std::queue<TimeStamp> oxts_ts_;
    std::queue<TimeStamp> velodyne_ts_;
    Eigen::Matrix3d R_i_in_v_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_i_in_v_ = Eigen::Vector3d::Zero();

    int oxts_cursor_ = 0;
    int velodyne_cursor_ = 0;
    TimeStamp last_sensor_ts_;
};

}  /* namespace loam */

#endif  /* LEGO_LOAM_MOD_UTILITY_KITTI_ADAPTER_HPP */
