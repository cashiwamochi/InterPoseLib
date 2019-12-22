#include <iostream>
#include <vector>

#include "InterPose.hpp"

int main(int argc, char* argv[])
{
  std::vector<Eigen::Matrix4f> v_se3_result, v_quat_t_result; 
  Eigen::Matrix4f pose_src = Eigen::Matrix4f::Identity(4,4);

  Eigen::Matrix4f pose_dst = Eigen::Matrix4f::Identity(4,4);
  pose_dst.block(0,0,3,3) = (Eigen::AngleAxisf(60.0*M_PI/180.0, Eigen::Vector3f::UnitZ())
                            * Eigen::AngleAxisf(45.0*M_PI/180.0, Eigen::Vector3f::UnitY())
                            * Eigen::AngleAxisf(30.0*M_PI/180.0, Eigen::Vector3f::UnitX()))
                            .normalized().toRotationMatrix();
  pose_dst.block(0,3,3,1) = Eigen::Vector3f{5.0, 10.0, 15.0};
  

  const int step_num = 30;
  SE3Interpolator se3_interpolator(step_num);
  // se3_interpolator.SetStepNum(step_num + 10); // You can change step_num after initialization
  v_se3_result = se3_interpolator.Interpolate(pose_src, pose_dst);

  QuatAndTInterpolator quat_t_interpolator(step_num);
  // quat_t_interpolator.SetStepNum(step_num + 10); // You can change step_num after initialization
  v_quat_t_result = quat_t_interpolator.Interpolate(pose_src, pose_dst);
  
  return 0;
}
