#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace InterPose {
namespace Utils {

  // 0 <= cos_theta <= 1, 0 <= theta <= pi
  const double EPS_to_0 = 1e-6;
  const double EPS_to_1 = 1.0 - 1e-6;

  Eigen::Matrix3d log(const Eigen::Matrix3d& R, double& theta) {
    Eigen::Matrix3d omega_x;

    const double cos_theta = (R.trace() - 1.0)/2.0; 

    if (cos_theta < EPS_to_0) { // need to handle theta = 0
      const double _theta = std::acos((R.trace() - 1.0)/2.0);
      const Eigen::Matrix3d _omega_x = 0.5*(1.0 + _theta*_theta/6.0)*(R - R.transpose()); // log(R)

      omega_x = _omega_x;
      theta = _theta;
    }
    else if(cos_theta > EPS_to_1) { // need to handle theta = pi
      Eigen::Matrix3d R_plus_I = R + Eigen::Matrix3d::Identity();
      int _col = -1;
      double longest_col_length = -1.0;
      for(int c = 0; c < 3; ++c) {
        double length = R_plus_I.col(c).norm();
        if(length > longest_col_length) {
          _col = c;
          longest_col_length = length;
        }

        theta = M_PI - EPS_to_0;
        const Eigen::Vector3d omega = (M_PI/longest_col_length)
                                      * Eigen::Vector3d(R_plus_I(0,_col), R_plus_I(1,_col), R_plus_I(2,_col));
        omega_x(0,0) = 0.0;       omega_x(0,1) = -omega(2); omega_x(0,2) = omega(1);
        omega_x(1,0) = omega(2);  omega_x(1,1) = 0;         omega_x(1,2) = -omega(0);
        omega_x(2,0) = -omega(1); omega_x(2,1) = omega(0);  omega_x(2,2) = 0.0;
      }
    }
    else { // usual case
      const double _theta = std::acos(cos_theta);
      const Eigen::Matrix3d _omega_x = 0.5*(R - R.transpose())/std::sin(_theta); // log(R)

      omega_x = _omega_x;
      theta = _theta;
    }

    return omega_x;
  }

  Eigen::Matrix4d exp(const Eigen::Vector3d& w, const Eigen::Vector3d& t, const double& theta) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    // theta = sqrt(w(0)*w(0) + w(1)*w(1) + w(2)*w(2));
    Eigen::Matrix3d w_x;
    w_x(0,0) = 0.0;   w_x(0,1) = -w(2); w_x(0,2) = w(1);
    w_x(1,0) = w(2);  w_x(1,1) = 0;     w_x(1,2) = -w(0);
    w_x(2,0) = -w(1); w_x(2,1) = w(0);  w_x(2,2) = 0.0;

    Eigen::Matrix3d e_w_x = Eigen::Matrix3d::Identity() 
                            + (std::sin(theta)/theta)*w_x
                            + ((1.0-std::cos(theta))/(theta*theta))*w_x*w_x;
    
    Eigen::Matrix3d V = Eigen::Matrix3d::Identity() 
                        + ((1.0-std::cos(theta))/(theta*theta))*w_x
                        + ((theta - std::sin(theta))/(theta*theta*theta))*w_x*w_x;

    Eigen::Vector3d _t = V * t;

    T.block(0,0,3,3) = e_w_x;
    T.block(0,3,3,1) = _t;

    return T;
  }
}
}
