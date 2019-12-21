#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

class PoseInterpolator {
  public:
    PoseInterpolator(const int& num_steps){SetStepNum(num_steps);};
    void SetStepNum(const int& num_steps){m_num_steps = num_steps;};

    virtual std::vector<Eigen::Matrix4d> Interpolate(const Eigen::Matrix4d& pose_src, const Eigen::Matrix4d& pose_dst) = 0;

  protected:
    int m_num_steps;
    Eigen::Matrix4d m_pose_src;
    Eigen::Matrix4d m_pose_dst;
};


class SE3Interpolator : public PoseInterpolator {
  public:
    SE3Interpolator(const int& num_steps = 3) : PoseInterpolator(num_steps) {};

    virtual std::vector<Eigen::Matrix4d> Interpolate(const Eigen::Matrix4d& pose_src, const Eigen::Matrix4d& pose_dst);
};

class QuatAndTInterpolator : public PoseInterpolator {
  public:
    QuatAndTInterpolator(const int& num_steps = 3) : PoseInterpolator(num_steps) {};

    virtual std::vector<Eigen::Matrix4d> Interpolate(const Eigen::Matrix4d& pose_src, const Eigen::Matrix4d& pose_dst);
};
