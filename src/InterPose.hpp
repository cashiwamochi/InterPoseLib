#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

class PoseInterpolator {
  public:
    PoseInterpolator(const int& num_steps){SetStepNum(num_steps);};
    void SetStepNum(const int& num_steps){m_num_steps = num_steps;};

    virtual std::vector<Eigen::Matrix4d> Interpolate(const Eigen::Matrix4d& pose_src, const Eigen::Matrix4d& pose_dst) = 0;
    virtual std::vector<Eigen::Matrix4f> Interpolate(const Eigen::Matrix4f& pose_src, const Eigen::Matrix4f& pose_dst) = 0;

  protected:
    int m_num_steps;
    Eigen::Matrix4d m_pose_src;
    Eigen::Matrix4d m_pose_dst;
};


class SE3Interpolator : public PoseInterpolator {
  public:
    SE3Interpolator(const int& num_steps = 3) : PoseInterpolator(num_steps) {};

    std::vector<Eigen::Matrix4d> Interpolate(const Eigen::Matrix4d& pose_src, const Eigen::Matrix4d& pose_dst);

    std::vector<Eigen::Matrix4f> Interpolate(const Eigen::Matrix4f& pose_src, const Eigen::Matrix4f& pose_dst) {
      std::vector<Eigen::Matrix4f> result; result.reserve(m_num_steps-1);

      Eigen::Matrix4d _pose_src = pose_src.cast<double>();
      Eigen::Matrix4d _pose_dst = pose_dst.cast<double>();
      std::vector<Eigen::Matrix4d> 
        v_poses = Interpolate(_pose_src, _pose_dst);

      for(Eigen::Matrix4d pose : v_poses) {
        result.push_back(pose.cast<float>());
      }
    
      return result;
    }

};

class QuatAndTInterpolator : public PoseInterpolator {
  public:
    QuatAndTInterpolator(const int& num_steps = 3) : PoseInterpolator(num_steps) {};

    std::vector<Eigen::Matrix4d> Interpolate(const Eigen::Matrix4d& pose_src, const Eigen::Matrix4d& pose_dst);

    std::vector<Eigen::Matrix4f> Interpolate(const Eigen::Matrix4f& pose_src, const Eigen::Matrix4f& pose_dst) {
      std::vector<Eigen::Matrix4f> result; result.reserve(m_num_steps-1);

      Eigen::Matrix4d _pose_src = pose_src.cast<double>();
      Eigen::Matrix4d _pose_dst = pose_dst.cast<double>();
      std::vector<Eigen::Matrix4d> 
        v_poses = Interpolate(_pose_src, _pose_dst);

      for(Eigen::Matrix4d pose : v_poses) {
        result.push_back(pose.cast<float>());
      }
    
      return result;
    }

};
