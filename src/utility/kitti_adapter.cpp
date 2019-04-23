#include "./kitti_adapter.hpp"

#include <iomanip>
#include <istream>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <vector>

#include <loguru.hpp>

namespace detail {

static const std::string kOXTSRelativePath     = "oxts/";
static const std::string kOXTSTSFile           = "timestamps.txt";
static const std::string kVelodyneRelativePath = "velodyne_points/";
static const std::string kVelodyneTSFile       = "timestamps_end.txt";
static const std::string kCalibIMUToVeloFile   = "calib_imu_to_velo.txt";

std::istream& SafelyGetLine(std::istream& is, std::string* str) {
    str->clear();
    std::streambuf* sb = is.rdbuf();
    while (true) {
        char ch = sb->sbumpc();
        switch (ch) {
            case '\n': return is;
            case '\r': if (sb->sgetc() == '\n')
                           sb->sbumpc();
                       return is;
            case EOF : if (str->empty())
                           is.setstate(std::istream::eofbit);
                       return is;
            default  : (*str) += ch;
        }
    }
}

}  /* namespace detail */

namespace loam {

KITTIAdapter::KITTIAdapter(
        const std::string& kitti_dir, const std::string& calib_dir)
    : oxts_path_(kitti_dir), velodyne_path_(kitti_dir)
{
    if (kitti_dir.back() != '/') {
        oxts_path_     += '/' + detail::kOXTSRelativePath;
        velodyne_path_ += '/' + detail::kVelodyneRelativePath;
    } else {
        oxts_path_     += detail::kOXTSRelativePath;
        velodyne_path_ += detail::kVelodyneRelativePath;
    }

    std::string imu_to_velo_filename = calib_dir;
    if (calib_dir.back() != '/') {
        imu_to_velo_filename += '/' + detail::kCalibIMUToVeloFile;
    } else {
        imu_to_velo_filename += detail::kCalibIMUToVeloFile;
    }

    std::tie(R_i_in_v_, p_i_in_v_) = GetCalibIMUToVelo(imu_to_velo_filename);

    oxts_ts_ = LoadTimeStampNanoseconds(oxts_path_+detail::kOXTSTSFile);
    velodyne_ts_ = LoadTimeStampNanoseconds(velodyne_path_+detail::kVelodyneTSFile);

    LOG_S(INFO) << "num OXTS: " << oxts_ts_.size();
    LOG_S(INFO) << "num Velodyne: " << velodyne_ts_.size();

    while (velodyne_ts_.front() < oxts_ts_.front()) {
        velodyne_ts_.pop();
        ++velodyne_cursor_;
    }

    last_sensor_ts_ = oxts_ts_.front();
}

void KITTIAdapter::OnNextOXTS(SensorCallbackType&& callback) {
    on_next_oxts_ = std::move(callback);
}

void KITTIAdapter::OnNextVelodyne(SensorCallbackType&& callback) {
    on_next_velodyne_ = std::move(callback);
}

bool KITTIAdapter::NextSensor() {
    unsigned char cond = 0b00;
    if (not oxts_ts_.empty())     cond += 0b01;
    if (not velodyne_ts_.empty()) cond += 0b10;

    switch (cond) {
        case 0b00: return false;
        case 0b01: return NextOXTS();
        case 0b10: return NextVelodyne();
        case 0b11: if (oxts_ts_.front() <= velodyne_ts_.front()) {
                       return NextOXTS();
                   } else {
                       return NextVelodyne();
                   }
    }

    /* this should never be reached */
    return false;
}

std::tuple<Eigen::Matrix3d, Eigen::Vector3d>
KITTIAdapter::GetCalibIMUToVelo(const std::string& calib_filename) {
    std::ifstream fis(calib_filename);
    CHECK_S(not fis.fail()) << "Failed to open " << calib_filename;

    std::string linestring;
    detail::SafelyGetLine(fis, &linestring);
    LOG_S(INFO) << "header: "<< linestring;

    Eigen::Matrix3d R;
    detail::SafelyGetLine(fis, &linestring);
    std::stringstream ss(linestring);
    std::string label;
    ss >> label;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            ss >> R(i, j);
    }
    LOG_S(INFO) << label << Eigen::Map<Eigen::Matrix<double, 1, 9>>(R.data());

    Eigen::Vector3d p;
    detail::SafelyGetLine(fis, &linestring);
    ss = std::stringstream(linestring);
    ss >> label;
    for (int i = 0; i < 3; ++i)
        ss >> p[i];
    LOG_S(INFO) << label << p.transpose();

    return std::make_tuple(R, p);
}

std::queue<KITTIAdapter::TimeStamp>
KITTIAdapter::LoadTimeStampNanoseconds(
        const std::string& timestamp_filename) {
    std::ifstream fis(timestamp_filename);
    CHECK_S(not fis.fail()) << "Failed to open " << timestamp_filename;
    std::queue<KITTIAdapter::TimeStamp> ts;
    std::string datestr, timestr;
    while (fis >> datestr >> timestr) {
        long s = 0;
        s += (timestr[0] - '0') * 60 * 60 * 10;
        s += (timestr[1] - '0') * 60 * 60;
        s += (timestr[3] - '0') * 60 * 10;
        s += (timestr[4] - '0') * 60;
        s += (timestr[6] - '0') * 10;
        s += (timestr[7] - '0');
        long ns = std::atoll(timestr.c_str() + 9);
        ts.emplace(s, ns);
    }
    return ts;
}

bool KITTIAdapter::NextOXTS() {
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << oxts_cursor_;
    std::string filename = oxts_path_ + "data/" + ss.str() + ".txt";
    std::ifstream ifs(filename);
    if (ifs.fail()) {
        ++oxts_cursor_;
        oxts_ts_.pop();
        return false;
    }

    std::vector<double> data;
    for (double v = 0.0; ifs >> v;)
        data.push_back(v);
    on_next_oxts_(
            data.data(),
            data.size(),
            oxts_ts_.front().second,
            oxts_ts_.front().nanosecond);

    ++oxts_cursor_;
    oxts_ts_.pop();
    last_sensor_ts_ = oxts_ts_.front();
    return true;
}

bool KITTIAdapter::NextVelodyne() {
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << velodyne_cursor_;
    std::string filename = velodyne_path_ + "data/" + ss.str() + ".txt";
    std::ifstream ifs(filename);
    if (ifs.fail()) {
        ++velodyne_cursor_;
        velodyne_ts_.pop();
        return false;
    }

    std::vector<double> data;
    for (double v = 0.0; ifs >> v;)
        data.push_back(v);
    for (int i = 0; i < (int)data.size(); i += 4) {
        Eigen::Map<Eigen::Vector3d> x(data.data() + i);
        x = R_i_in_v_.transpose() * (x - p_i_in_v_);
    }
    on_next_velodyne_(
            data.data(),
            data.size(),
            oxts_ts_.front().second,
            oxts_ts_.front().nanosecond);

    ++velodyne_cursor_;
    velodyne_ts_.pop();
    last_sensor_ts_ = velodyne_ts_.front();
    return true;
}

}  /* namespace loam */
