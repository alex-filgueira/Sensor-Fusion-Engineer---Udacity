#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false; // -> false

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
  

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2; 

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 6;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. Msee ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  //My init
  is_initialized_ = false;

  n_x_ = 5;

  n_aug_ = 7;

  lambda_ = 6;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  time_us_ = 0.0;
  // End my init

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if(is_initialized_ == false)
  {
    if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      x_ << meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]), meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[2]),0,0,0;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
  }

  double delta_t = static_cast<double>((meas_package.timestamp_ - time_us_) * 1e-6);
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  if(meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }
  else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

   VectorXd Vx_aug = VectorXd(n_aug_);
   MatrixXd Mp_aug = MatrixXd(n_aug_, n_aug_);
   MatrixXd Mx_sig = MatrixXd(n_aug_, 2*n_aug_+1);

   Vx_aug.head(5) = x_;
   Vx_aug(5)=0.0;
   Vx_aug(6)=0.0;

   Mx_sig.col(0) = Vx_aug;

   Mp_aug.fill(0.0);
   Mp_aug.topLeftCorner(5,5)= P_;
   Mp_aug(5,5) = std_a_*std_a_;
   Mp_aug(6,6) = std_yawdd_*std_yawdd_;

   MatrixXd Ml = Mp_aug.llt().matrixL();

   for(int i = 0; i < n_aug_; ++i){
    Mx_sig.col(i + 1)          = Vx_aug + sqrt(lambda_ + n_aug_) * Ml.col(i);
    Mx_sig.col(i + 1 + n_aug_) = Vx_aug - sqrt(lambda_ + n_aug_) * Ml.col(i);
  }

  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    double dp_x = Mx_sig(0, i);
    double dp_y = Mx_sig(1, i);
    double dv = Mx_sig(2, i);
    double dyaw = Mx_sig(3, i);
    double dyawd = Mx_sig(4, i);
    double dnu_a = Mx_sig(5, i);
    double dnu_yawdd = Mx_sig(6, i);

    double dpx_p;
    double dyawd_p;
    double dyaw_p;
    double dpy_p;
    double dv_p;

    if(fabs(dyawd) > 0.001)
    {
      dpx_p = dp_x + dv/dyawd*(sin(dyaw+dyawd*delta_t) - sin(dyaw)) + 0.5*delta_t*delta_t*cos(dyaw)*dnu_a;
      dpy_p = dp_y + dv/dyawd*(-cos(dyaw+dyawd*delta_t) + cos(dyaw)) + 0.5*delta_t*delta_t*sin(dyaw)*dnu_a;
      dv_p = dv + delta_t*dnu_a;
      dyaw_p = dyaw + dyawd*delta_t + 0.5*delta_t*delta_t*dnu_yawdd;
      dyawd_p = dyawd + delta_t*dnu_yawdd;
    }
    else
    {
      dpx_p = dp_x + dv*cos(dyaw)*delta_t + 0.5*delta_t*delta_t*cos(dyaw)*dnu_a;
      dpy_p = dp_y + dv*sin(dyaw)*delta_t + 0.5*delta_t*delta_t*sin(dyaw)*dnu_a;
      dv_p = dv + delta_t*dnu_a;
      dyaw_p = dyaw + dyawd*delta_t + 0.5*delta_t*delta_t*dnu_yawdd;
      dyawd_p = dyawd + delta_t*dnu_yawdd;
    }
    Xsig_pred_(0, i) = dpx_p;
    Xsig_pred_(1, i) = dpy_p;
    Xsig_pred_(2, i) = dv_p;
    Xsig_pred_(3, i) = dyaw_p;
    Xsig_pred_(4, i) = dyawd_p;
  }

  VectorXd V_x = VectorXd::Zero(5);
  MatrixXd M_p = MatrixXd::Zero(5, 5);

  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    V_x += weights_(i) * Xsig_pred_.col(i);
  }

  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    VectorXd VVx_diff = Xsig_pred_.col(i) - V_x;

    while(VVx_diff(3) > M_PI){
      VVx_diff(3) -= 2.0*M_PI;
    }
    while(VVx_diff(3) < -M_PI){
      VVx_diff(3) += 2.0*M_PI;
    }

    M_p += weights_(i) * VVx_diff * VVx_diff.transpose();
  }
  x_ = V_x;
  P_ = M_p;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIMs, if desired.
   */
  int n_z_ = 2;
  MatrixXd Mzsig = MatrixXd(n_z_, 2*n_aug_+1);

  VectorXd Vz_pred = VectorXd(n_z_);

  MatrixXd Ms = MatrixXd(n_z_, n_z_);

  for(int i =0; i< 2*n_aug_+1;++i){
    Mzsig(0,i) =  Xsig_pred_(0,i);
    Mzsig(1,i) = Xsig_pred_(1,i);
  }

  Vz_pred.fill(0.0);
  for(int i=0; i<2*n_aug_+1; ++i){
    Vz_pred = Vz_pred + weights_(i)*Mzsig.col(i);
  }

  Ms.fill(0.0);
  for(int i=0; i<2*n_aug_+1; ++i){
    VectorXd Vz_diff = Mzsig.col(i) - Vz_pred;

    while(Vz_diff(1)>M_PI){
      Vz_diff(1)-=2.*M_PI;
    }
    while(Vz_diff(1)<-M_PI){
      Vz_diff(1)+=2.*M_PI;
    }

    Ms = Ms + weights_(i)*Vz_diff*Vz_diff.transpose();
  }
   MatrixXd MR = MatrixXd(n_z_,n_z_);
   MR<< std_laspx_*std_laspx_,0,
        0,std_laspy_*std_laspy_;
  Ms = Ms + MR;

  MatrixXd Mtc = MatrixXd(n_x_,n_z_);
  Mtc.fill(0.0);
  for(int i=0; i<2*n_aug_+1; ++i){
    VectorXd Vz_diff = Mzsig.col(i) - Vz_pred;

    while(Vz_diff(1)>M_PI){
      Vz_diff(1)-=2.*M_PI;
    }
    while(Vz_diff(1)<-M_PI){
      Vz_diff(1)+=2.*M_PI;
    }

    VectorXd Vx_diff = Xsig_pred_.col(i)-x_;

    while(Vx_diff(1)>M_PI){
      Vx_diff(1)-=2.*M_PI;
    }
    while(Vx_diff(1)<-M_PI){
      Vx_diff(1)+=2.*M_PI;
    }

    Mtc = Mtc + weights_(i)*Vx_diff*Vz_diff.transpose();
  }

  MatrixXd MK = Mtc * Ms.inverse();
  VectorXd Vz=meas_package.raw_measurements_;
  VectorXd Vz_diff = Vz - Vz_pred;

  while(Vz_diff(1)>M_PI){
    Vz_diff(1)-=2.*M_PI;
  }
  while(Vz_diff(1)<-M_PI){
    Vz_diff(1)+=2.*M_PI;
  }

  x_ = x_ + MK*Vz_diff;
  P_ = P_ - MK*Ms*MK.transpose();

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIMs, if desired.
   */

  int n_z_ = 3;
  MatrixXd Mzsig = MatrixXd(n_z_, 2*n_aug_+1);

  VectorXd Vz_pred =VectorXd(n_z_);

  MatrixXd Ms = MatrixXd(n_z_, n_z_);

  for(int i =0; i< 2*n_aug_+1;++i){
    double dpx = Xsig_pred_(0,i);
    double dpy = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double phi = Xsig_pred_(3,i);
    double phid = Xsig_pred_(4,i);
    double v1 = cos(phi)*v;
    double v2 = sin(phi)*v;

    Mzsig(0,i) = sqrt(dpx*dpx +dpy*dpy);
    Mzsig(1,i) = atan2(dpy,dpx);
    Mzsig(2,i) = (dpx*v1 + dpy*v2)/ sqrt(dpx*dpx + dpy*dpy);
  }

  Vz_pred.fill(0.0);
  for(int i=0; i<2*n_aug_+1; ++i){
    Vz_pred = Vz_pred + weights_(i) * Mzsig.col(i);
  }

  Ms.fill(0.0);
  for(int i=0; i<2*n_aug_+1; ++i){
    VectorXd Vz_diff = Mzsig.col(i) - Vz_pred;

    while(Vz_diff(1)>M_PI){
      Vz_diff(1) -= 2. * M_PI;
    }
    while(Vz_diff(1)<-M_PI){
      Vz_diff(1) += 2. * M_PI;
    }

    Ms = Ms + weights_(i) * Vz_diff * Vz_diff.transpose();
  }
   MatrixXd MR = MatrixXd(n_z_,n_z_);
   MR<< std_radr_*std_radr_,0,0,
        0,std_radphi_*std_radphi_,0,
        0,0,std_radrd_*std_radrd_;
  Ms = Ms + MR;

  MatrixXd Mtc = MatrixXd(n_x_,n_z_);
  Mtc.fill(0.0);
  for(int i=0; i<2*n_aug_+1; ++i){
    VectorXd Vz_diff = Mzsig.col(i) - Vz_pred;

    while(Vz_diff(1)>M_PI){
      Vz_diff(1) -= 2. * M_PI;
    }
    while(Vz_diff(1)<-M_PI){
      Vz_diff(1) += 2. * M_PI;
    }

    VectorXd Vx_diff = Xsig_pred_.col(i)-x_;

    while(Vx_diff(1)>M_PI){
      Vx_diff(1) -= 2. * M_PI;
    }
    while(Vx_diff(1)<-M_PI){
      Vx_diff(1) += 2. * M_PI;
    }

    Mtc = Mtc + weights_(i) * Vx_diff * Vz_diff.transpose();
  }

  MatrixXd MK = Mtc *Ms.inverse();
  VectorXd Vz = meas_package.raw_measurements_;
  VectorXd Vz_diff = Vz-Vz_pred;

  while(Vz_diff(1)>M_PI){
    Vz_diff(1) -= 2. * M_PI;
  } 
  while(Vz_diff(1)<-M_PI){
    Vz_diff(1) += 2. * M_PI;
  }

  x_ = x_ + MK * Vz_diff;
  P_ = P_ - MK * Ms * MK.transpose();
}