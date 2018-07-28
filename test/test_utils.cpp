#include <gtest/gtest.h>

#include <barrier-certs/LimitedSensing.h>

namespace bc = barrier_certs;

template <class Func>
double der(double v1, double v2, Func func) {
    return (func(v1) - func(v2)) / (v1 - v2);
}

template <class Func>
void check_derivatives(double v, double eps, Func func) {

    const double der1 = der(v + eps, v + 2 * eps, func);
    const double der2 = der(v - eps, v - 2 * eps, func);
    const double der3 = der(v - eps, v + eps, func);

    // larger eps for test because of division
    const double der_eps = 1.0e3 * eps;
    EXPECT_NEAR(der1, der2, der_eps);
    EXPECT_NEAR(der1, der3, der_eps);
}

TEST(utilities, psi) {
    const double xi = 1;
    const double beta = 0.5;
    const double c = beta * xi;
    const double eps = 1.0e-9;

    bc::LimitedSensing limited_sensing;
    limited_sensing.set_beta(beta);
    auto psi = [&](double val) {return limited_sensing.psi(xi, val);};

    // check endpoints
    EXPECT_NEAR(psi(c), c, eps);

    check_derivatives(c, eps, psi);
    check_derivatives(xi, eps, psi);
    EXPECT_NEAR(der(c - eps, c + eps, psi), 1, 1e3 * eps);
    EXPECT_NEAR(der(xi - eps, xi + eps, psi), 0, 1e3 * eps);
}
