#include <iostream>
#include "Viewer.hpp"

Viewer::Viewer()
  : m_pose_setting{180.0, 180.0, 180.0, 50.0, 50.0, 50.0, 40}
{
  mb_is_initialized = false;

  m_Twc_src = Eigen::Matrix4d::Identity(4,4);

  m_se3_interpolator.SetStepNum(m_pose_setting.max_num_steps/2);
  m_quat_t_interpolator.SetStepNum(m_pose_setting.max_num_steps/2);
}

Viewer::Viewer(const PoseSetting& _pose_setting) 
  : m_pose_setting(_pose_setting)
{
  mb_is_initialized = false;

  m_Twc_src = Eigen::Matrix4d::Identity(4,4);

  m_se3_interpolator.SetStepNum(m_pose_setting.max_num_steps/2);
  m_quat_t_interpolator.SetStepNum(m_pose_setting.max_num_steps/2);
}

void Viewer::Run() {
  /*INITIALIZATION*/
  pangolin::CreateWindowAndBind("InterPoseViewer", WIN_WIDTH, WIN_HEIGHT);
  glEnable(GL_DEPTH_TEST);

  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(
    pangolin::ProjectionMatrix(WIN_WIDTH,WIN_HEIGHT,420,420,WIN_WIDTH/2,WIN_HEIGHT/2,0.2,1000),
    pangolin::ModelViewLookAt(0,3,-3, 0,0,0, pangolin::AxisY)
    // pangolin::ModelViewLookAt(0,10,15, 0,0,0, 0.0,-1.0,0.0)
  );

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay()
                          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -(float)WIN_WIDTH/(float)WIN_HEIGHT)
                          .SetHandler(&handler);

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

  // If true, interpolation is done and poses are shown
  pangolin::Var<bool> ui_button_update("ui.UPDATE",false,false);
  pangolin::Var<bool> ui_button_reset("ui.RESET",false,false);

  pangolin::Var<double>
    ui_rot_x("ui.RotX",m_pose_setting.max_rot_x/2,-m_pose_setting.max_rot_x,m_pose_setting.max_rot_x); // screenname, initial value, min, max
  pangolin::Var<double>
    ui_rot_y("ui.RotY",m_pose_setting.max_rot_y/2,-m_pose_setting.max_rot_y,m_pose_setting.max_rot_y);
  pangolin::Var<double>
    ui_rot_z("ui.RotZ",m_pose_setting.max_rot_z/2,-m_pose_setting.max_rot_z,m_pose_setting.max_rot_z);
  pangolin::Var<double>
    ui_trans_x("ui.TransX",m_pose_setting.max_trans_x/2,-m_pose_setting.max_trans_x,m_pose_setting.max_trans_x);
  pangolin::Var<double>
    ui_trans_y("ui.TransY",m_pose_setting.max_trans_y/2,-m_pose_setting.max_trans_y,m_pose_setting.max_trans_y);
  pangolin::Var<double>
    ui_trans_z("ui.TransZ",m_pose_setting.max_trans_z/2,-m_pose_setting.max_trans_z,m_pose_setting.max_trans_z);

  // Select modes you want to draw
  pangolin::Var<bool> ui_check_se3("ui.SE3",false,true);
  pangolin::Var<bool> ui_check_q_t("ui.Quat+t",false,true);

  pangolin::Var<int> ui_num_steps("ui.NumSteps",m_pose_setting.max_num_steps/2,5,m_pose_setting.max_num_steps);

  pangolin::Var<bool> ui_check_axis_or_frustm("ui.DrawAxisOrFrustum",false,true);

  glClearColor(0.07f, 0.2f, 0.2f, 0.0f);

  pangolin::OpenGlMatrix Twc_src; Eigen2Gl(m_Twc_src, Twc_src);
  pangolin::OpenGlMatrix Twc_dst; Eigen2Gl(m_Twc_dst, Twc_dst);

  std::vector<Eigen::Matrix4d> v_Twc_QuatT;
  std::vector<Eigen::Matrix4d> v_Twc_SE3;
    
  while( !pangolin::ShouldQuit() )
  {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    if(pangolin::Pushed(ui_button_reset)) {
      ui_rot_x = 0.0; ui_rot_y = 0.0; ui_rot_z = 0.0;
      ui_trans_x = 0.0; ui_trans_y = 0.0; ui_trans_z = 0.0;
    }

    // update camera pose params
    if(pangolin::Pushed(ui_button_update) || pangolin::Pushed(ui_button_reset) || !mb_is_initialized) {
      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity(4,4);
      pose.block(0,0,3,3) = (Eigen::AngleAxisd(ui_rot_z*M_PI/180.0, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(ui_rot_y*M_PI/180.0, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(ui_rot_x*M_PI/180.0, Eigen::Vector3d::UnitX()))
                            .normalized().toRotationMatrix();
      pose.block(0,3,3,1) = Eigen::Vector3d{ui_trans_x, ui_trans_y, ui_trans_z};

      m_Twc_dst = pose;
      mb_is_initialized = true;
    }

    // Draw src and dst cameras
    {
      std::vector<Eigen::Matrix4d> v_Twc{m_Twc_src, m_Twc_dst};
      for(auto Twc_eig : v_Twc){
        pangolin::OpenGlMatrix Twc_gl;
        Eigen2Gl(Twc_eig, Twc_gl);

        glPushMatrix();
        Twc_gl.Multiply();
        DrawCamera(m_src_dst_frustum_color, ui_check_axis_or_frustm);
        glPopMatrix();
      }
    }
    

    // Draw interpolated poses using Quarternion + R^3
    if(ui_check_q_t) {
      m_quat_t_interpolator.SetStepNum(ui_num_steps); 
      v_Twc_QuatT = m_quat_t_interpolator.Interpolate(m_Twc_src, m_Twc_dst);

      for(auto Twc_eig : v_Twc_QuatT){
        pangolin::OpenGlMatrix Twc_gl;
        Eigen2Gl(Twc_eig, Twc_gl);

        glPushMatrix();
        Twc_gl.Multiply();
        DrawCamera(m_q_t_frustum_color, ui_check_axis_or_frustm);
        glPopMatrix();
      }
    }

    // Draw interpolated poses using SE3
    if(ui_check_se3) {
      m_se3_interpolator.SetStepNum(ui_num_steps); 
      v_Twc_SE3 = m_se3_interpolator.Interpolate(m_Twc_src, m_Twc_dst);

      for(auto Twc_eig : v_Twc_SE3){
        pangolin::OpenGlMatrix Twc_gl;
        Eigen2Gl(Twc_eig, Twc_gl);

        glPushMatrix();
        Twc_gl.Multiply();
        DrawCamera(m_se3_frustum_color, ui_check_axis_or_frustm);
        glPopMatrix();
      }
    }
    
    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return;
}

void Viewer::DrawCamera(const float* color, const bool& axis_or_frustum) {
  if(axis_or_frustum) {
    DrawCameraAxis(color);
  }
  else {
    DrawCameraFrustum(color);
  }

  return;
}

void Viewer::DrawCameraFrustum(const float* color) const {
  glLineWidth(1);
  glColor3f(0.0f,1.0f,1.0f);
  glColor3f(color[0],color[1],color[2]);
  glBegin(GL_LINES);

  glVertex3f(0,0,0);
  glVertex3f(FRUSTUM_WIDTH,FRUSTUM_HEIGHT,FRUSTUM_DEPTH);

  glVertex3f(0,0,0);
  glVertex3f(FRUSTUM_WIDTH,-FRUSTUM_HEIGHT,FRUSTUM_DEPTH);

  glVertex3f(0,0,0);
  glVertex3f(-FRUSTUM_WIDTH,-FRUSTUM_HEIGHT,FRUSTUM_DEPTH);

  glVertex3f(0,0,0);
  glVertex3f(-FRUSTUM_WIDTH,FRUSTUM_HEIGHT,FRUSTUM_DEPTH);

  glVertex3f(FRUSTUM_WIDTH,FRUSTUM_HEIGHT,FRUSTUM_DEPTH);
  glVertex3f(FRUSTUM_WIDTH,-FRUSTUM_HEIGHT,FRUSTUM_DEPTH);

  glVertex3f(-FRUSTUM_WIDTH,FRUSTUM_HEIGHT,FRUSTUM_DEPTH);
  glVertex3f(-FRUSTUM_WIDTH,-FRUSTUM_HEIGHT,FRUSTUM_DEPTH);

  glVertex3f(-FRUSTUM_WIDTH,FRUSTUM_HEIGHT,FRUSTUM_DEPTH);
  glVertex3f(FRUSTUM_WIDTH,FRUSTUM_HEIGHT,FRUSTUM_DEPTH);

  glVertex3f(-FRUSTUM_WIDTH,-FRUSTUM_HEIGHT,FRUSTUM_DEPTH);
  glVertex3f(FRUSTUM_WIDTH,-FRUSTUM_HEIGHT,FRUSTUM_DEPTH);

  glEnd();
  return;
}

// not implemented 
void Viewer::DrawCameraAxis(const float* color) const {
  glLineWidth(1);
  glBegin(GL_LINES);

  glColor3f(0.0f,0.0f,1.0f);
  glVertex3f(0,0,0);
  glVertex3f(AXIS_LENGTH,0,0);

  glColor3f(0.0f,1.0f,0.0f);
  glVertex3f(0,0,0);
  glVertex3f(0,AXIS_LENGTH,0);

  glColor3f(1.0f,0.0f,0.0f);
  glVertex3f(0,0,0);
  glVertex3f(0,0,AXIS_LENGTH);

  glEnd();

  glPointSize(10);
  glBegin(GL_POINTS);
  glColor3f(color[0], color[1], color[2]);
  glVertex3f(0.0, 0.0, 0.0);
  glEnd();

  return;
}
