//  control_system_solver.hpp
//  Created by TH on 2020/11/29.
//  Copyright © 2020 TH. All rights reserved.

#ifndef CONTROL_SYSTEM_SOLVER_H_
#define CONTROL_SYSTEM_SOLVER_H_

#include <array>
#include <cassert>
#include <functional>
#include <iostream>
#include <tuple>

#include "solver_method.hpp"

namespace solver {

enum Solver {
  EULER
};

template <class T>
struct TimeConfig {
  T start_time;
  T end_time;
  T delta_t;
};

template <class T, Solver SOLVER, std::size_t DATA_LENGTH, std::size_t N>
class ControlSystemSolver {
 public:
  explicit ControlSystemSolver(const TimeConfig<T>& time_config,
                               const std::array<T, N>& initial_value)
    : time_config_(time_config),
      initial_value_(initial_value),
      is_save_data_(false) {}

  // Solve control system: dx = f(t, x, u), u = k(t, x).
  // Where t in R^1, x in R^N, u in R^M, f: R^1 x R^N x R^M -> R^N, k: R^1 x R^N -> R^M.
  template <std::size_t M>
  std::tuple<std::array<std::array<T, DATA_LENGTH>, 1>,
             std::array<std::array<T, DATA_LENGTH>, N>,
             std::array<std::array<T, DATA_LENGTH>, M>> Solve(std::function<std::array<T, N>(const T, const std::array<T, N>&, const std::array<T, M>&)> f,
                                                              std::function<std::array<T, M>(const T, const std::array<T, N>&)> k) {
    std::array<std::array<T, DATA_LENGTH>, M> u_data {};
    std::size_t data_i = 0;

    std::array<T, M> u {};

    auto [t_data, x_data] = SolveODEWithFixedStep([&](const T t, const std::array<T, N>& x) {
      u = k(t, x);

      if (is_save_data_) {
        for (std::size_t j = 0; j < u.size(); j++) {
          u_data[j][data_i] = u[j];
        }

        data_i++;
      }

      return f(t, x, u);
    });

    return { t_data, x_data, u_data };
  }

 private:
  TimeConfig<T> time_config_;
  std::array<T, N> initial_value_;
  bool is_save_data_;

  // Solve ODE: dx = f(t, x).
  // Where t in R^1, x in R^N, f: R^1 x R^N -> R^N.
  std::tuple<std::array<std::array<T, DATA_LENGTH>, 1>,
             std::array<std::array<T, DATA_LENGTH>, N>> SolveODEWithFixedStep(std::function<std::array<T, N>(const T, const std::array<T, N>&)> f) {
    assert(time_config_.end_time > time_config_.start_time);
    assert(time_config_.delta_t > 0);

    const auto n = static_cast<std::size_t>(std::round((time_config_.end_time - time_config_.start_time) / time_config_.delta_t));
    assert(n >= DATA_LENGTH);

    std::array<std::array<T, DATA_LENGTH>, 1> t_data {};
    std::array<std::array<T, DATA_LENGTH>, N> x_data {};
    std::size_t data_i = 0;

    T t = time_config_.start_time;
    std::array<T, N> x = initial_value_;
    std::array<T, N> dx {};

    for (std::size_t i = 0; i < n; i++) {
      if ((i % (n / DATA_LENGTH)) == 0) {
        t_data[0][data_i] = t;
        for (std::size_t j = 0; j < x.size(); j++) {
          x_data[j][data_i] = x[j];
        }
        data_i++;
        is_save_data_ = true;
      } else {
        is_save_data_ = false;
      }

      t += time_config_.delta_t;

      if constexpr (SOLVER == EULER)
        dx = SolverMethod::Euler<T, N>(time_config_.delta_t, t, x, f);

      for (std::size_t j = 0; j < x.size(); j++) {
        x[j] += dx[j];
      }
    }

    return { t_data, x_data };
  }
};

} // namespace solver

#endif