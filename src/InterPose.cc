#include "InterPose.hpp"
#include "Utils.hpp"

using namespace Eigen;

std::vector<Matrix4d> SE3Interpolator::Interpolate(const Matrix4d& pose_src, const Matrix4d& pose_dst) {
  std::vector<Matrix4d> v_interpolated_poses(m_num_steps-1, Matrix4d::Identity());

  // S(pecial)E(uclidian)(3) Interpolation
  // SE3Up = T0*exp(beta*log(T0^-1 * T1))
  for(int step = 1; step < m_num_steps; ++step) {
    const double beta = 1.0 - static_cast<double>(step)/static_cast<double>(m_num_steps);
    // _T = T0^-1 * T1
    Eigen::Matrix4d _T = pose_src.inverse() * pose_dst;

    // SE(3) -> se(3)
    {
      Eigen::Matrix3d w_x;
      // SO(3) R -> so(3) w_x (and theta)
      double theta = 0.0;
      w_x = InterPose::Utils::log(_T.block(0,0,3,3), theta);

      // SE(3) -> se(3)
      Eigen::Matrix3d V = Eigen::Matrix3d::Identity() 
                          + ((1.0-std::cos(theta))/(theta*theta))*w_x
                          + ((theta - std::sin(theta))/(theta*theta*theta))*w_x*w_x;
  
      Eigen::Vector3d t_ = V.inverse() * _T.block(0,3,3,1);
      Eigen::Vector3d w(w_x(2,1), w_x(0,2), w_x(1,0));

      t_ *= beta;
      w *= beta;
      theta *= beta;

      Eigen::Matrix4d T = InterPose::Utils::exp(w, t_, theta);

      Eigen::Matrix4d TT = pose_src * T;
      v_interpolated_poses[step-1] = TT;
    }
  }

  return v_interpolated_poses;
}

std::vector<Matrix4d> QuatAndTInterpolator::Interpolate(const Matrix4d& pose_src, const Matrix4d& pose_dst) {
  std::vector<Matrix4d> v_interpolated_poses(m_num_steps-1, Matrix4d::Identity());

  // Translation Linear Interpolation
  {
    const Vector3d t_src = pose_src.block(0,3,3,1);
    const Vector3d t_dst = pose_dst.block(0,3,3,1);
    
    for(int step = 1; step < m_num_steps; ++step) {
      const double beta = static_cast<double>(step)/static_cast<double>(m_num_steps);
      Vector3d t = (t_dst - t_src)*beta + t_src;

      v_interpolated_poses[step-1].block(0,3,3,1) = t; 
    }
  }

  // Rotation Spherical Interpolation(Quarternion)
  // Algorithm -> https://www.mathworks.com/help/fusion/ref/quaternion.slerp.html
  {
    Quaterniond quat_src(Matrix3d(pose_src.block(0,0,3,3))); quat_src.normalize();
    Quaterniond quat_dst(Matrix3d(pose_dst.block(0,0,3,3))); quat_dst.normalize();
  
    for(int step = 1; step < m_num_steps; ++step) {
      const double beta = static_cast<double>(step)/static_cast<double>(m_num_steps);
      const double theta = std::acos(quat_src.dot(quat_dst));
  
      Vector4d _q
        = ((std::sin((1.0-beta)*theta)/std::sin(theta))*Vector4d(quat_src.w(),quat_src.x(),quat_src.y(),quat_src.z())
          + (std::sin(beta*theta)/std::sin(theta))*Vector4d(quat_dst.w(),quat_dst.x(),quat_dst.y(),quat_dst.z()));
      _q.normalize();
  
      const Quaterniond q(_q(0),_q(1),_q(2),_q(3)); 
  
      v_interpolated_poses[step-1].block(0,0,3,3) = q.toRotationMatrix(); 
    }
  }

  return v_interpolated_poses;
}
