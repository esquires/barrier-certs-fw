
#include <barrier-certs/LimitedSensing.h>
#include <barrier-certs/Utilities.h>

#include <scrimmage/math/Angles.h>

#include <utility>
#include <iostream>

namespace barrier_certs {

LimitedSensing::LimitedSensing(std::map<std::string, std::string> &params) {
    beta_ = std::stod(params.at("beta"));
    sensing_range_ = std::stod(params.at("sensing_range"));
    if (sensing_range_ < 0) sensing_range_ = std::numeric_limits<double>::infinity();
    delta_ = scrimmage::Angles::deg2rad(std::stod(params.at("delta")));
    Ds_ = std::stod(params.at("Ds"));

    auto[calc_type, fw_params] = parse_fw_params(params);

    vel_fw_ = fw_params.gamma[0];
    omega_fw_ = fw_params.gamma[1];
}

double LimitedSensing::calc_xi() {
    const double radius = vel_fw_ / omega_fw_;
    double out = sqrt(pow(sensing_range_ - 4 * radius, 2) - 4 * delta_) - Ds_;
    return out;
}

double LimitedSensing::tilde_h(double h) {
    const double xi = calc_xi();

    if (h <= beta_ * xi) {
        return h;
    } else if (h <= xi) {
        return psi(xi, h);
    } else {
        return psi(xi, xi);
    }
}

double LimitedSensing::psi(double xi, double val) {
    const double c1 = -1 / (2 * (1 - beta_) * xi);
    const double c2 = -2 * xi * c1;
    const double c3 = beta_ * xi - c1 * pow(beta_ * xi, 2) - c2 * beta_ * xi;

    const double out = c1 * pow(val, 2) + c2 * val + c3;
    return out;
}

} // namespace barrier_certs
