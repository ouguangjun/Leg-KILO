// SPDX-License-Identifier: MIT
// @file kernel.hpp
// @brief Robust loss kernels for residual reweighting.
#ifndef LEG_KILO_COMMON_KERNEL_HPP_
#define LEG_KILO_COMMON_KERNEL_HPP_

#include <algorithm>
#include <cmath>

namespace legkilo {

class RobustKernel {
   public:
    explicit RobustKernel(double delta) : delta_(std::max(delta, kMinDelta)) {}
    virtual ~RobustKernel() = default;

    double delta() const { return delta_; }
    virtual double weight(double squared_error) const = 0;

   protected:
    static constexpr double kMinDelta = 1e-6;
    double delta_ = 1.0;
};

class HuberKernel final : public RobustKernel {
   public:
    explicit HuberKernel(double delta) : RobustKernel(delta) {}

    double weight(double squared_error) const override {
        const double error = std::sqrt(std::max(0.0, squared_error));
        return error <= delta_ ? 1.0 : delta_ / error;
    }
};

class CauchyKernel final : public RobustKernel {
   public:
    explicit CauchyKernel(double delta) : RobustKernel(delta) {}

    double weight(double squared_error) const override {
        const double normalized_error = std::max(0.0, squared_error) / (delta_ * delta_);
        return 1.0 / (1.0 + normalized_error);
    }
};

class TukeyKernel final : public RobustKernel {
   public:
    explicit TukeyKernel(double delta) : RobustKernel(delta) {}

    double weight(double squared_error) const override {
        const double normalized_error = std::max(0.0, squared_error) / (delta_ * delta_);
        if (normalized_error >= 1.0) return 0.0;
        const double residual = 1.0 - normalized_error;
        return residual * residual;
    }
};

}  // namespace legkilo

#endif  // LEG_KILO_COMMON_KERNEL_HPP_
