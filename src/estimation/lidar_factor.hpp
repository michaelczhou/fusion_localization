#ifndef LOAM_MOD_LASER_LIDAR_FACTOR_HPP
#define LOAM_MOD_LASER_LIDAR_FACTOR_HPP

#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include "./math.hpp"

namespace loam {

class PointEdgeFactor : public ceres::SizedCostFunction<3, 10, 10> {
public:
    PointEdgeFactor(
            const double x_i_L_k_1[3],
            const double x_j_L_k[3],
            const double x_l_L_k[3])
        :
            x_i_L_k_1_(x_i_L_k_1),
            x_j_L_k_(x_j_L_k),
            x_l_L_k_(x_l_L_k)
    { }

    virtual bool Evaluate(
            double const* const* two_states,
            double* residuals,
            double** jacobians) const override {
        using Eigen::Map;
        using Eigen::Vector3d;
        using Eigen::Matrix3d;
        using Eigen::Quaterniond;
        using RowMatrix3x10d = Eigen::Matrix<double, 3, 10, Eigen::RowMajor>;

        Map<const Quaterniond> q_L_k(two_states[0]+0);
        Map<const Vector3d>    p_L_k(two_states[0]+4);

        Map<const Quaterniond> q_L_k_1(two_states[1]+0);
        Map<const Vector3d>    p_L_k_1(two_states[1]+4);

        Vector3d x_i_L_k = q_L_k.conjugate() * (q_L_k_1 * x_i_L_k_1_ + p_L_k_1 - p_L_k);
        Vector3d line_j_l = x_j_L_k_ - x_l_L_k_;
        Vector3d line_i_l = x_i_L_k - x_l_L_k_;
        Vector3d line_i_j = x_i_L_k - x_j_L_k_;

        Map<Vector3d>(residuals).noalias() = line_i_j.cross(line_i_l) / line_j_l.norm();

        if (jacobians) {
            Matrix3d J_x_i_L_k = SkewSymmMatrix(-line_j_l) / line_j_l.norm();

            if (jacobians[0]) {
                Map<RowMatrix3x10d> mapped_jacobian(jacobians[0]);
                mapped_jacobian.leftCols(3) = J_x_i_L_k * SkewSymmMatrix(x_i_L_k);
                mapped_jacobian.middleCols(3, 3) = -J_x_i_L_k;
                mapped_jacobian.rightCols(4).setZero();
            }

            if (jacobians[1]) {
                Quaterniond q_k_k_1 = q_L_k.conjugate() * q_L_k_1;

                Map<RowMatrix3x10d> mapped_jacobian(jacobians[1]);
                mapped_jacobian.leftCols(3) = -J_x_i_L_k * q_k_k_1.matrix() * SkewSymmMatrix(x_i_L_k_1_);
                mapped_jacobian.middleCols(3, 3) = J_x_i_L_k * q_k_k_1.matrix();
                mapped_jacobian.rightCols(4).setZero();
            }
        }

        return true;
    }

private:
    /// \brief Point x_i in (k+1)-th frame (a.k.a. the lastest frame).
    const Eigen::Vector3d x_i_L_k_1_;

    /// \brief Point x_j in k-th frame.
    const Eigen::Vector3d x_j_L_k_;

    /// \brief Point x_l in k-th frame.
    const Eigen::Vector3d x_l_L_k_;
};

class PointPlanarFactor : public ceres::SizedCostFunction<3, 10, 10> {
public:
    PointPlanarFactor(
            const double x_i_L_k_1[3],
            const double x_j_L_k[3],
            const double x_l_L_k[3],
            const double x_m_L_k[3])
        :
            x_i_L_k_1_(x_i_L_k_1),
            x_j_L_k_(x_j_L_k),
            x_l_L_k_(x_l_L_k),
            x_m_L_k_(x_m_L_k)
    { }

    virtual bool Evaluate(double const* const* two_states,
                          double* residuals,
                          double** jacobians) const override {
        using Eigen::Map;
        using Eigen::Vector3d;
        using Eigen::Matrix3d;
        using Eigen::Quaterniond;
        using RowMatrix3x10d = Eigen::Matrix<double, 3, 10, Eigen::RowMajor>;

        Map<const Quaterniond> q_L_k(two_states[0]+0);
        Map<const Vector3d>    p_L_k(two_states[0]+4);

        Map<const Quaterniond> q_L_k_1(two_states[1]);
        Map<const Vector3d>    p_L_k_1(two_states[1]+4);

        Vector3d x_i_L_k = q_L_k.conjugate() * (q_L_k_1 * x_i_L_k_1_ + p_L_k_1 - p_L_k);
        Vector3d a = x_i_L_k - x_j_L_k_;
        Vector3d b = (x_j_L_k_ - x_l_L_k_).cross(x_j_L_k_ - x_m_L_k_);

        Map<Vector3d>(residuals).noalias() = b * b.transpose() * a / b.squaredNorm();

        if (jacobians) {
            Matrix3d J_x_i_L_k = b * b.transpose() / b.squaredNorm();

            if (jacobians[0]) {
                Map<RowMatrix3x10d> mapped_jacobian(jacobians[0]);
                mapped_jacobian.leftCols(3) = J_x_i_L_k * SkewSymmMatrix(x_i_L_k);
                mapped_jacobian.middleCols(3, 3) = -J_x_i_L_k;
                mapped_jacobian.rightCols(4).setZero();
            }

            if (jacobians[1]) {
                Quaterniond q_k_k_1 = q_L_k.conjugate() * q_L_k_1;

                Map<RowMatrix3x10d> mapped_jacobian(jacobians[1]);
                mapped_jacobian.leftCols(3) = -J_x_i_L_k * q_k_k_1.matrix() * SkewSymmMatrix(x_i_L_k_1_);
                mapped_jacobian.middleCols(3, 3) = J_x_i_L_k * q_k_k_1.matrix();
                mapped_jacobian.rightCols(4).setZero();
            }
        }

        return true;
    }

private:
    /// \brief Point x_i in (k+1)-th frame (a.k.a. the lastest frame).
    const Eigen::Vector3d x_i_L_k_1_;

    /// \brief Point x_j in k-th frame.
    const Eigen::Vector3d x_j_L_k_;

    /// \brief Point x_l in k-th frame.
    const Eigen::Vector3d x_l_L_k_;

    /// \brief Point x_m in k-th frame.
    const Eigen::Vector3d x_m_L_k_;
};

}  /* namespace loam */

struct LidarPlaneNormFactor
{

    LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
                         double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
                                                         negative_OA_dot_norm(negative_OA_dot_norm_) {}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = q_w_curr * cp + t_w_curr;

        Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
        residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
                                       const double negative_OA_dot_norm_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarPlaneNormFactor, 1, 4, 3>(
                new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
};

struct LidarDistanceFactor
{

    LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_)
            : curr_point(curr_point_), closed_point(closed_point_){}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = q_w_curr * cp + t_w_curr;


        residual[0] = point_w.x() - T(closed_point.x());
        residual[1] = point_w.y() - T(closed_point.y());
        residual[2] = point_w.z() - T(closed_point.z());
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarDistanceFactor, 3, 4, 3>(
                new LidarDistanceFactor(curr_point_, closed_point_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d closed_point;
};

struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 3, 4, 3>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

struct LidarPlaneFactor
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
									   const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

#endif  /* LOAM_MOD_LASER_LIDAR_FACTOR_HPP */
