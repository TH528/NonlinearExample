//  solver_method.hpp
//  Created by TH on 2020/11/29.
//  Copyright © 2020 TH. All rights reserved.

#ifndef SOLVER_METHOD_H_
#define SOLVER_METHOD_H_

#include <array>
#include <functional>

namespace solver {

class SolverMethod {
 public:
  template <class T, std::size_t N>
  static std::array<T, N> Euler(const T dt,
                                const T t,
                                const std::array<T, N>& x,
                                std::function<std::array<T, N>(const T, const std::array<T, N>&)> f) {
    std::array<T, N> dx {};

    const auto rhs = f(t, x);

    for (std::size_t i = 0; i < dx.size(); i++) {
      dx[i] = rhs[i] * dt;
    }

    return dx;
  }

 private:
  SolverMethod() {}
};

} // namespace solver

#endif