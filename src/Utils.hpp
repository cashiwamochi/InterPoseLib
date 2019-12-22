#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace InterPose {
namespace Utils {

  // -1 <= cos_theta <= 1, 0 <= theta <= pi
  const double EPS_to_0 = 1e-5;
  const double EPS_to_1 = 1.0 - 1e-5;

  template <typename TYPE>
  Eigen::Matrix<TYPE,3,3> log(const Eigen::Matrix<TYPE,3,3>& R, TYPE& theta) {
    Eigen::Matrix<TYPE,3,3> omega_x;

    const TYPE cos_theta = (R.trace() - 1.0)/2.0; 

    if(cos_theta < -EPS_to_1) { // need to handle theta = pi
      Eigen::Matrix<TYPE,3,3> R_plus_I = R + Eigen::Matrix<TYPE,3,3>::Identity();
      int _col = -1;
      TYPE longest_col_length = -1.0;
      for(int c = 0; c < 3; ++c) {
        TYPE length = R_plus_I.col(c).norm();
        if(length > longest_col_length) {
          _col = c;
          longest_col_length = length;
        }

        theta = M_PI;
        const Eigen::Matrix<TYPE,3,1> omega = (M_PI/longest_col_length)
                                          * Eigen::Matrix<TYPE,3,1>(R_plus_I(0,_col), R_plus_I(1,_col), R_plus_I(2,_col));
        omega_x(0,0) = 0.0;       omega_x(0,1) = -omega(2); omega_x(0,2) = omega(1);
        omega_x(1,0) = omega(2);  omega_x(1,1) = 0;         omega_x(1,2) = -omega(0);
        omega_x(2,0) = -omega(1); omega_x(2,1) = omega(0);  omega_x(2,2) = 0.0;
      }
    }
    else if (cos_theta > EPS_to_1) { // need to handle theta = 0
      const TYPE _theta = std::acos((R.trace() - 1.0)/2.0);
      const Eigen::Matrix<TYPE,3,3> _omega_x = 0.5*(1.0 + _theta*_theta/6.0)*(R - R.transpose()); // log(R)

      omega_x = _omega_x;
      theta = _theta;
    }
    else { // usual case
      const TYPE _theta = std::acos(cos_theta);
      const Eigen::Matrix<TYPE,3,3> _omega_x = _theta*(R - R.transpose())/(2.0*std::sin(_theta)); // log(R)

      omega_x = _omega_x;
      theta = _theta;
    }

    return omega_x;
  }

  template <typename TYPE>
  Eigen::Matrix<TYPE,4,4> exp(const Eigen::Matrix<TYPE,3,1>& w, const Eigen::Matrix<TYPE,3,1>& t, const TYPE& theta) {
    Eigen::Matrix<TYPE,4,4> T = Eigen::Matrix4d::Identity();
    Eigen::Matrix<TYPE,3,3> w_x;
    w_x(0,0) = 0.0;   w_x(0,1) = -w(2); w_x(0,2) = w(1);
    w_x(1,0) = w(2);  w_x(1,1) = 0;     w_x(1,2) = -w(0);
    w_x(2,0) = -w(1); w_x(2,1) = w(0);  w_x(2,2) = 0.0;

    if(theta < EPS_to_0) {
      Eigen::Matrix<TYPE,3,3> e_w_x = Eigen::Matrix<TYPE,3,3>::Identity(); 
      T.block(0,0,3,3) = e_w_x;
      T.block(0,3,3,1) = e_w_x * t;
    }
    else {
      Eigen::Matrix<TYPE,3,3> e_w_x = Eigen::Matrix<TYPE,3,3>::Identity() 
                              + (std::sin(theta)/theta)*w_x
                              + ((1.0-std::cos(theta))/(theta*theta))*w_x*w_x;
      
      Eigen::Matrix<TYPE,3,3> V = Eigen::Matrix<TYPE,3,3>::Identity() 
                          + ((1.0-std::cos(theta))/(theta*theta))*w_x
                          + ((theta - std::sin(theta))/(theta*theta*theta))*w_x*w_x;

      Eigen::Matrix<TYPE,3,1> _t = V * t;

      T.block(0,0,3,3) = e_w_x;
      T.block(0,3,3,1) = _t;
    }

    return T;
  }
}
}
