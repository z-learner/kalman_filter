#include "kalman_filter/linear_kalman.hpp"
#include "matplotlibcpp.h"

#include <chrono>
#include <random>

namespace plt = matplotlibcpp;


// uset wo-dimensional uniformly accelerated linear motion
int main(int argc, char** argv) {
  double ax      = 0.1;
  double ay      = 0.2;
  double delta_t = 0.01;

  Eigen::Matrix<double, 4, 4> Transpose{};
  Transpose << 1, 0, delta_t, 0, 0, 1, 0, delta_t, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::Matrix<double, 4, 2> Control{};
  Control << 0.5 * delta_t * delta_t, 0, 0, 0.5 * delta_t * delta_t, delta_t, 0, 0, delta_t;

  Eigen::Matrix<double, 2, 1> u;
  u << ax, ay;

  // genrate data
  unsigned                         seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine       generator(seed);
  std::normal_distribution<double> distribution(0.0, 0.01);



  std::vector<Eigen::Matrix<double, 4, 1>> real_datas{};
  std::vector<Eigen::Matrix<double, 4, 1>> meature_datas{};
  real_datas.push_back(Eigen::Matrix<double, 4, 1>::Zero());
  meature_datas.push_back(Eigen::Matrix<double, 4, 1>::Zero());

  for (size_t index = 0; index < 1000; ++index) {
    Eigen::Matrix<double, 4, 1> new_real_value = Transpose * real_datas[index] + Control * u;
    Eigen::Matrix<double, 4, 1> noise;
    noise << distribution(generator), distribution(generator), distribution(generator), distribution(generator);
    Eigen::Matrix<double, 4, 1> new_meaturement_value = new_real_value + noise;

    // std::cout << "new_real_value         :" << new_real_value.transpose() << std::endl;
    // std::cout << "new_meaturement_value  :" << new_meaturement_value.transpose() << std::endl;
    // std::cout << noise.transpose() << std::endl;
    real_datas.push_back(new_real_value);
    meature_datas.push_back(new_meaturement_value);
  }

  // R is bigger than Q, believe predict value
  kalman_filter::LinearKalmanFilter<4, 4, 2> linear_kf(Eigen::Matrix<double, 4, 4>::Identity() * 1e-2, Eigen::Matrix<double, 4, 4>::Identity() * 1e-3);
  linear_kf.UpdateTransitionMatrix(Transpose);
  linear_kf.UpdateControlMatrix(Control);
  linear_kf.UpdateMeatruementMatrix(Eigen::Matrix<double, 4, 4>::Identity());

  std::vector<Eigen::Matrix<double, 4, 1>> predict_datas{};
  predict_datas.push_back(linear_kf.State());

  for (size_t index = 0; index < 1000; ++index) {
    linear_kf.Predict(u);
    linear_kf.Correct(meature_datas[index + 1]);
    predict_datas.push_back(linear_kf.State());
  }

  // draw
  std::vector<double> real_x;
  std::vector<double> real_y;
  std::vector<double> meature_x;
  std::vector<double> meature_y;
  std::vector<double> kf_x;
  std::vector<double> kf_y;

  std::vector<double> real_meatrue_x;
  std::vector<double> real_meatrue_y;

  std::vector<double> real_kf_x;
  std::vector<double> real_kf_y;

  std::vector<double> indexs;
  for (size_t index = 0; index <= 1000; ++index) {
    indexs.push_back(index);
    real_x.push_back(real_datas[index][0]);
    real_y.push_back(real_datas[index][1]);

    // std::cout << "real_x : " << real_x[index] << std::endl;
    // std::cout << "real_y : " << real_y[index] << std::endl;

    meature_x.push_back(meature_datas[index][0]);
    meature_y.push_back(meature_datas[index][1]);

    kf_x.push_back(predict_datas[index][0]);
    kf_y.push_back(predict_datas[index][1]);

    real_meatrue_x.push_back(real_datas[index][0] - meature_datas[index][0]);
    real_meatrue_y.push_back(real_datas[index][1] - meature_datas[index][1]);


    real_kf_x.push_back(real_datas[index][0] - predict_datas[index][0]);
    real_kf_y.push_back(real_datas[index][1] - predict_datas[index][1]);
  }


  plt::named_plot("meature", indexs, meature_x);
  plt::named_plot("real", indexs, real_x);
  plt::named_plot("kf", indexs, kf_x);
  plt::title("Sample Position x figure");
  plt::legend();
  plt::save("./positionx.png");
  plt::close();

  plt::named_plot("meature", indexs, meature_y);
  plt::named_plot("real", indexs, real_y);
  plt::named_plot("kf", indexs, kf_y);
  plt::title("Sample Position x figure");
  plt::legend();
  plt::save("./positiony.png");
  plt::close();

  plt::named_plot("real_meatrue", indexs, real_meatrue_x);
  plt::named_plot("real_kf", indexs, real_kf_x);
  plt::title("Sample loss figure");
  plt::legend();
  plt::save("./lossx.png");
  plt::close();

  plt::named_plot("real_meatrue", indexs, real_meatrue_y);
  plt::named_plot("real_kf", indexs, real_kf_y);
  plt::title("Sample loss figure");
  plt::legend();
  plt::save("./lossy.png");
  plt::close();





  return 0;
}