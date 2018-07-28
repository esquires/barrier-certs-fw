
#ifndef INCLUDE_BARRIER_CERTS_HELPERTYPES_H_
#define INCLUDE_BARRIER_CERTS_HELPERTYPES_H_

#include <vector>
#include <utility>

namespace barrier_certs {

struct VehicleParams {
    std::vector<std::pair<double, double>> lims;
    std::vector<double> gamma;
};

struct ControlsData {
    double t;
    size_t veh_id;
    std::vector<double> u_hat;
    std::vector<double> u_safe;
};

struct SafetyData {
    double t;
    size_t veh_id;

    double min_d;
    size_t min_d_id;

    double min_h;
    size_t min_h_id;
};

enum class CalcType {STRAIGHT = 0, TURN};

enum class VehicleType {FixedWing};

} // namespace barrier_certs
#endif // INCLUDE_BARRIER_CERTS_HELPERTYPES_H_
