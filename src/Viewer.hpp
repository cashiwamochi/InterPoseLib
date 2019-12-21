#include <vector>

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include "InterPose.hpp"

class Viewer {
  public:
    struct PoseSetting {
      const double max_rot_x;
      const double max_rot_y;
      const double max_rot_z;
      const double max_trans_x;
      const double max_trans_y;
      const double max_trans_z;
      const int max_num_steps;
    };

    // enum CamDrawType {
    //   FRUSTUM = 0,
    //   AXIS = 1,
    // };

  public:
    Viewer();
    Viewer(const PoseSetting& _setting);
    ~Viewer(){};

    void Run();
  private:
    void Eigen2Gl(const Eigen::Matrix4d pose_eig, pangolin::OpenGlMatrix& pose_gl) {
      for(int c_gl = 0; c_gl < 4; ++c_gl) {
        for(int r_gl = 0; r_gl < 4; ++r_gl) {
          pose_gl.m[c_gl + 4*r_gl] = pose_eig(c_gl, r_gl);
        }
      }
    };
    /* 0 1 2 3
     * 4 5 6 7
     * 8 9 10 11
     * 12 13 14 15
     *   | |
     *   V V
     * 0 4 8  12
     * 1 5 9  13
     * 2 6 10 14
     * 3 7 11 15
     */

    void DrawCamera(const float* color, const bool& axis_or_frustum);
    void DrawCameraFrustum(const float* color) const;
    void DrawCameraAxis() const;

  private:
    SE3Interpolator m_se3_interpolator;
    QuatAndTInterpolator m_quat_t_interpolator;

    const int UI_WIDTH = 180;
    const int WIN_WIDTH = 640;
    const int WIN_HEIGHT = 480;

    const int FRUSTUM_WIDTH = 4; 
    const int FRUSTUM_HEIGHT = 3; 
    const int FRUSTUM_DEPTH = 2; 

    const int AXIS_LENGTH = 2;

    const PoseSetting m_pose_setting;

    const float m_q_t_frustum_color[3] = {0.0, 1.0, 1.0};
    const float m_se3_frustum_color[3] = {0.9, 0.9, 0.1};
    const float m_src_dst_frustum_color[3] = {0.8, 0.1, 0.1};

    bool mb_is_initialized;
    // CamDrawType m_draw_type;

    Eigen::Matrix4d m_Twc_src;
    Eigen::Matrix4d m_Twc_dst;
};
