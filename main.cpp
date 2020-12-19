//  main.cpp
//  Created by TH on 2020/11/29.
//  Copyright © 2020 TH. All rights reserved.

#include <cmath>
#include <iostream>

#include "solver/control_system_solver.hpp"
#include "csv/csv_writer.hpp"

int main() {
  constexpr solver::TimeConfig<double> TimeConfig {
    0.0, // start_time
    30.0, // end_time
    1E-4 // delta_t
  };

  static constexpr std::size_t N = 7; // state dimmension
  static constexpr std::size_t M = 3; // input dimmension + saving data dimmesion

  constexpr std::array<double, 3> J { 1.0, 1.0, 1.0 };
  constexpr double Alpha = 1.0;
  constexpr std::array<double, 3> Gamma { 1.0, 1.0, 1.0 };

  // Controller
  auto k = [&](const double t, const std::array<double, N>& x) {
    std::array<double, 4> q { x[0], x[1], x[2], x[3] };
    std::array<double, 3> omega { x[4], x[5], x[6] };

    const std::array<double, 7> dVdx {
      -2 * Alpha,
      -Gamma[0] * (omega[0] + Gamma[0] * q[1]),
      -Gamma[1] * (omega[1] + Gamma[1] * q[2]),
      -Gamma[2] * (omega[2] + Gamma[2] * q[3]),
      omega[0] + Gamma[0] * q[1],
      omega[1] + Gamma[1] * q[2],
      omega[2] + Gamma[2] * q[3]
    };

    const std::array<double, 7> fx {
      -0.5 * (q[1] * omega[0] + q[2] * omega[1] + q[3] * omega[2]),
      0.5 * (q[0] * omega[0] - q[3] * omega[1] + q[2] * omega[2]),
      0.5 * (q[3] * omega[0] + q[0] * omega[1] - q[1] * omega[2]),
      0.5 * (-q[2] * omega[0] + q[1] * omega[1] + q[0] * omega[2]),
      (J[1] - J[2]) / J[0] * omega[1] * omega[2],
      (J[2] - J[0]) / J[1] * omega[0] * omega[2],
      (J[0] - J[1]) / J[2] * omega[0] * omega[1]
    };

    std::array<std::array<double, 3>, 7> gx {};
    gx[4][0] = 1.0 / J[0];
    gx[5][1] = 1.0 / J[1];
    gx[6][2] = 1.0 / J[2];

    double LfV = 0;
    for (int i = 0; i < N; i++) {
      LfV += dVdx[i] * fx[i];
    }

    std::array<double, 3> LgV {};
    for (int j = 0; j < M; j++) {
      for (int i = 0; i < N; i++) {
        LgV[j] += dVdx[i] * gx[i][j];
      }
    }

    double norm_LgV = 0;
    double sum = 0;
    for (int j = 0; j < M; j++) {
      sum += std::pow(LgV[j], 2.0);
    }
    norm_LgV = std::sqrt(sum);

    std::array<double, M> u {};

    if (norm_LgV != 0.0) {
      for (int j = 0; j < M; j++) {
        u[j] = -(LfV + std::sqrt(std::pow(LfV, 2.0) + std::pow(norm_LgV, 4.0))) / (std::pow(norm_LgV, 2.0)) * LgV[j];
      }
    }

    return u;
  };

  // Control system
  auto f = [&](const double t, const std::array<double, N>& x, const std::array<double, M>& u) {
    // Right hand side of control system
    std::array<double, 4> q { x[0], x[1], x[2], x[3] };
    std::array<double, 3> omega { x[4], x[5], x[6] };

    std::array<double, N> dx {
      -0.5 * (q[1] * omega[0] + q[2] * omega[1] + q[3] * omega[2]),
      0.5 * (q[0] * omega[0] - q[3] * omega[1] + q[2] * omega[2]),
      0.5 * (q[3] * omega[0] + q[0] * omega[1] - q[1] * omega[2]),
      0.5 * (-q[2] * omega[0] + q[1] * omega[1] + q[0] * omega[2]),
      (J[1] - J[2]) / J[0] * omega[1] * omega[2] + u[0] / J[0],
      (J[2] - J[0]) / J[1] * omega[0] * omega[2] + u[1] / J[1],
      (J[0] - J[1]) / J[2] * omega[0] * omega[1] + u[2] / J[2]
    };

    return dx;
  };

  constexpr std::array<double, 3> r { 0.5, 0.183013, 0.5 };
  const double r0 = std::sqrt(1.0 - std::pow(r[0], 2.0) - std::pow(r[1], 2.0) - std::pow(r[2], 2.0));
  const std::array<double, N> X0 { r0, r[0], r[1], r[2], -1.5708, 3.14159, -1.5708 }; // Initial state

  solver::ControlSystemSolver<double, solver::Solver::EULER, 1000, N> control_system_solver(TimeConfig, X0);
  auto [t_data, x_data, u_data] = control_system_solver.Solve<M>(f, k);

  csv::CSVWriter<double> t_csv_data("t_data.csv");
  csv::CSVWriter<double> x_csv_data("x_data.csv");
  csv::CSVWriter<double> u_csv_data("u_data.csv");

  t_csv_data.Open();
  x_csv_data.Open();
  u_csv_data.Open();

  t_csv_data.Write(t_data);
  x_csv_data.Write(x_data);
  u_csv_data.Write(u_data);

  return 0;
}