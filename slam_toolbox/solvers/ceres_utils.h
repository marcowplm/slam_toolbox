/*
 * Copyright 2018 Simbe Robotics
 * Author: Steve Macenski
 */

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <cmath>
#include <utility>
#include <karto_sdk/Karto.h>

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline std::size_t GetHash(const int& x, const int& y)
{
  return ((std::hash<double>()(x) ^ (std::hash<double>()(y) << 1)) >> 1);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Normalizes the angle in radians between [-pi and pi).
template <typename T> inline T NormalizeAngle(const T& angle_radians)
{
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Defines a local parameterization for updating the angle to be constrained in [-pi to pi).
class AngleLocalParameterization
{
 public:
  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians, T* theta_radians_plus_delta) const
  {
    *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
    return true;
  }

  static ceres::LocalParameterization* Create()
  {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
  }
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians)
{
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);
  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement.
//
// residual =  information^{1/2} * [  r_a^T * (p_b - p_a) - \hat{p_ab}   ]
//                                 [ Normalize(yaw_b - yaw_a - \hat{yaw_ab}) ]
//
// where r_a is the rotation matrix that rotates a vector represented in frame A
// into the global frame, and Normalize(*) ensures the angles are in the range
// [-pi, pi).
class PoseGraph2dErrorTerm
{
 public:
  PoseGraph2dErrorTerm(double x_ab, double y_ab, double yaw_ab_radians, const Eigen::Matrix3d& sqrt_information)
      : p_ab_(x_ab, y_ab), 
        yaw_ab_radians_(yaw_ab_radians), 
        sqrt_information_(sqrt_information)
  {
  }

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a, 
                  const T* const x_b, const T* const y_b, const T* const yaw_b, 
                  T* residuals_ptr) const
  {
    const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
    const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);
    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals_map(residuals_ptr);
    residuals_map.template head<2>() = RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) - p_ab_.cast<T>();
    residuals_map(2) = NormalizeAngle((*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));
    // Scale the residuals by the square root information matrix to account for the measurement uncertainty.
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;
    return true;
  }

  static ceres::CostFunction* Create(double x_ab, double y_ab, double yaw_ab_radians, const Eigen::Matrix3d& sqrt_information) 
  {
    return (new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1, 1, 1>(
      new PoseGraph2dErrorTerm(x_ab, y_ab, yaw_ab_radians, sqrt_information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The position of B relative to A in the A frame.
  const Eigen::Vector2d p_ab_;
  // The orientation of frame B relative to frame A.
  const double yaw_ab_radians_;
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Defines a pose 3D (position + quaternion) in the CeresSolver format
struct CeresPose3d 
{
  Eigen::Vector3d p;
  Eigen::Quaterniond q;

  CeresPose3d() 
  {
  }

  /**
   * Construct a CeresPose3d object from a karto::Pose3
   */ 
  CeresPose3d(const karto::Pose3& rPose) :
    p(rPose.GetPosition().GetX(), rPose.GetPosition().GetY(), rPose.GetPosition().GetZ()),
    q(rPose.GetOrientation().GetW(), rPose.GetOrientation().GetX(), rPose.GetOrientation().GetY(), rPose.GetOrientation().GetZ())
  {
    // Normalize the quaternion to account for precision loss due to serialization.
    q.normalize();
  }

  /**
   * Constructs a CeresPose3d object from a karto::Pose2
   */
  CeresPose3d(const karto::Pose2& rPose) :
    p(rPose.GetX(), rPose.GetY(), 0.0),
    q(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(rPose.GetHeading(), Eigen::Vector3d::UnitZ()))
  // NON serve normalizzare il quaternione perchè è già normalizzato per costruzione!
  // Il quaternione costruito in questo modo è corretto! Se si prova ad estrarre la matrice di rotazione e 
  // lo Heading (o Yaw) con GetEulerHeading() si ottiene l'angolo della Pose2 iniziale!
  {
  }

  /**
   * Constructs the quaternion from the 3 Euler angles
   * @param roll (rotation along x axis)
   * @param pitch (rotation along y axis)
   * @param yaw (rotation along z axis)
   */
  void FromEulerAngles(const kt_double roll, const kt_double pitch, const kt_double yaw)  
  {
    q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  }

  /**
   * Converts the orientation stored in the quaternion in Euler angles
   * @param rRoll (rotation along x axis)
   * @param rPitch (rotation along y axis)
   * @param RYaw (rotation along z axis)
   */
  void ToEulerAngles(kt_double& rRoll, kt_double& rPitch, kt_double& rYaw) const
  {
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    rRoll = euler(0);
    rPitch = euler(1);
    rYaw = euler(2);
  }    

  /**
   * Extract the Yaw (or Heading) from the quaternion
   * @return heading
   */
  kt_double GetEulerHeading() const
  {
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    if (euler(0) != 0 || euler(1) != 0)
    {
      std::cout << "\n\nGetEulerHeading: Pitch e/o Roll sono diversi da zero! "
                << "Questo non dovrebbe succedere!!" << std::endl
                << "Roll: "  << euler(0)             << std::endl
                << "Pitch: " << euler(1)             << std::endl;
    }            

    return euler(2); 
  }
  
};

// TODO: Queste due typedef dovrebbero essere in toolbox_types.hpp
typedef std::unordered_map<int, CeresPose3d>::iterator GraphIterator3d;
typedef std::unordered_map<int, CeresPose3d>::const_iterator ConstGraphIterator3d;


/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement. We have two poses x_a
// and x_b. Through sensor measurements we can measure the transformation of
// frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
// between the current estimate of the poses and the measurement.
//
// In this formulation, we have chosen to represent the rigid transformation as
// a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
// [x, y, z, w].
//
// The estimated measurement is:
//      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
//             [ q_ab ]    [ q_a^{-1] * q_b         ]
//
// where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
// quaternion. Now we can compute an error metric between the estimated and
// measurement transformation. For the orientation error, we will use the
// standard multiplicative error resulting in:
//
//   error = [ p_ab - \hat{p}_ab                 ]
//           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
//
// where Vec(*) returns the vector (imaginary) part of the quaternion. Since
// the measurement has an uncertainty associated with how accurate it is, we
// will weight the errors by the square root of the measurement information
// matrix:
//
//   residuals = I^{1/2) * error
// where I is the information matrix which is the inverse of the covariance.
class PoseGraph3dErrorTerm 
{
 public:
  PoseGraph3dErrorTerm(const CeresPose3d& t_ab_measured,
                       const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) 
  {
  }

  template <typename T>
  bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,
                  const T* const p_b_ptr, const T* const q_b_ptr,
                  T* residuals_ptr) const
  {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

    // Compute the relative transformation between the two frames.
    Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

    // Represent the displacement between the two frames in the A frame.
    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

    // Compute the error between the two orientation estimates.
    Eigen::Quaternion<T> delta_q =
        t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        p_ab_estimated - t_ab_measured_.p.template cast<T>();
    residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

  static ceres::CostFunction* Create(
      const CeresPose3d &t_ab_measured,
      const Eigen::Matrix<double, 6, 6> &sqrt_information)
  {
    return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
        new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The measurement for the position of B relative to A in the A frame.
  const CeresPose3d t_ab_measured_;
  // The square root of the measurement information matrix.
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};


/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Definisce una local parameterization per quaternioni che rappresentano orientamenti 2d
// Non sono affatto sicuro che sia corretta!!
class QuaternionAngleLocalParameterization
{
 public:
  template <typename T>
  bool operator()(const T* x, const T* delta_theta, T* x_plus_theta) const
  {
    // Estrae l'angolo theta (yaw / heading) dal quaternione
    T test = x[0] * x[1] + x[2] * x[3];
    T theta;
    if (test > T(0.499))
    {
      // singularity at north pole
      theta = T(2.0) * atan2(x[0], x[3]);
    }
    else if (test < -T(0.499))
    {
      // singularity at south pole
      theta = -T(2.0) * atan2(x[0], x[3]);
    }
    else
    {
      theta = atan2(T(2.0) * x[1] * x[3] - T(2.0) * x[0] * x[2], T(1.0) - T(2.0) * x[1] * x[1] - T(2.0) * x[2] * x[2]);
    }

    T two_pi(2.0 * M_PI);
    T theta_plus_delta_theta; 
    // la somma tra gli angoli è copiata da NormalizeAngle - N.B. delta_theta è un puntatore ad un vettore di dimensione 1 (che è la Local Size)
    theta_plus_delta_theta = (theta + delta_theta[0]) - two_pi * ceres::floor(((theta + delta_theta[0]) + T(M_PI)) / two_pi);
    // la costruzione del quaternione è copiata da karto::FromEulerAngles()
    x_plus_theta[0] = T(0.0); 
    x_plus_theta[1] = sin(theta_plus_delta_theta * T(0.5)); 
    x_plus_theta[2] = T(0.0); 
    x_plus_theta[3] = cos(theta_plus_delta_theta * T(0.5));
    
    return true;
  }

  static ceres::LocalParameterization* Create()
  {
    return (new ceres::AutoDiffLocalParameterization<QuaternionAngleLocalParameterization, 4, 1>);
  }
};
