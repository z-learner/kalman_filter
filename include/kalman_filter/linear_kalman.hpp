#pragma once

#include <Eigen/Dense>

namespace kalman_filter {

template<int kStateDim, int kMeasureDim, int kControlDim> class LinearKalmanFilter {
public:
  LinearKalmanFilter(const Eigen::Matrix<double, kMeasureDim, kMeasureDim>& meaturement_noise_variance, const Eigen::Matrix<double, kStateDim, kStateDim>& system_noise_variance,
                     const Eigen::Matrix<double, kStateDim, kStateDim>& error_cov_post, const Eigen::Matrix<double, kStateDim, 1> init_state)
    : meaturement_noise_variance_{meaturement_noise_variance}
    , system_noise_variance_{system_noise_variance}
    , error_cov_post_{error_cov_post}
    , last_state_{init_state} {}

  LinearKalmanFilter(const Eigen::Matrix<double, kMeasureDim, kMeasureDim>& meaturement_noise_variance, const Eigen::Matrix<double, kStateDim, kStateDim>& system_noise_variance)
    : LinearKalmanFilter(meaturement_noise_variance, system_noise_variance, Eigen::Matrix<double, kStateDim, kStateDim>::Identity(), Eigen::Matrix<double, kStateDim, 1>::Zero()) {}

  ~LinearKalmanFilter() = default;

  void UpdateTransitionMatrix(const Eigen::Matrix<double, kStateDim, kStateDim>& transition) { transition_ = transition; }
  void UpdateControlMatrix(const Eigen::Matrix<double, kStateDim, kControlDim>& control) { control_ = control; }
  void UpdateMeatruementMatrix(const Eigen::Matrix<double, kMeasureDim, kStateDim>& meaturement) { meaturement_ = meaturement; }

  const Eigen::Matrix<double, kStateDim, 1>& State() { return last_state_; }

  Eigen::Matrix<double, kStateDim, 1> Predict(const Eigen::Matrix<double, kControlDim, 1>& control_value) {
    last_state_     = transition_ * last_state_ + control_ * control_value;
    error_cov_post_ = transition_ * error_cov_post_ * error_cov_post_.transpose() + system_noise_variance_;
    return last_state_;
  }

  bool Correct(const Eigen::Matrix<double, kMeasureDim, 1>& meaturement_value) {
    // (kStateDim, kMeasureDim) = (kStateDim, kStateDim) x (kStateDim, kMeasureDim) * ( (kMeasureDim, kStateDim) *  (kStateDim, kStateDim) * (kStateDim, kMeasureDim) + (kMeasureDim, kMeasureDim)  )
    auto system_gain = error_cov_post_ * meaturement_.transpose() * (meaturement_ * error_cov_post_ * meaturement_.transpose() + meaturement_noise_variance_).inverse();
    // (kStateDim, 1) = (kStateDim, 1) + (kStateDim, kMeasureDim) * ( (kMeasureDim, 1) - ( kMeasureDim, kStateDim ) * (kStateDim, 1)
    last_state_     = last_state_ + system_gain * (meaturement_value - meaturement_ * last_state_);
    error_cov_post_ = (Eigen::Matrix<double, kStateDim, kStateDim>::Identity() - system_gain * meaturement_) * error_cov_post_;
    return true;
  }

private:
  Eigen::Matrix<double, kStateDim, kStateDim>     transition_{};
  Eigen::Matrix<double, kStateDim, kControlDim>   control_{};
  Eigen::Matrix<double, kMeasureDim, kStateDim>   meaturement_{};
  Eigen::Matrix<double, kMeasureDim, kMeasureDim> meaturement_noise_variance_{};
  Eigen::Matrix<double, kStateDim, kStateDim>     system_noise_variance_{};
  Eigen::Matrix<double, kStateDim, kStateDim>     error_cov_post_{};   // coveriance
  Eigen::Matrix<double, kStateDim, 1>             last_state_{};
};

}   // namespace kalman_filter