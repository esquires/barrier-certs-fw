
#ifndef INCLUDE_BARRIER_CERTS_PLUGINS_METRICS_BARRIERCERTMETRICS_BARRIERCERTMETRICS_H_
#define INCLUDE_BARRIER_CERTS_PLUGINS_METRICS_BARRIERCERTMETRICS_BARRIERCERTMETRICS_H_

#include <barrier-certs/HelperTypes.h>

#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/common/Visibility.h>

#include <pybind11/pybind11.h>

#include <map>
#include <string>
#include <utility>

namespace barrier_certs {
namespace metrics {

class DLL_PUBLIC BarrierCertMetrics : public scrimmage::Metrics {
 public:
    BarrierCertMetrics() = default;
    void init(std::map<std::string, std::string> &params) override;
    bool step_metrics(double /*t*/, double /*dt*/) override {return true;}
    void print_team_summaries() override;

 protected:
    pybind11::list data_controls_;
    pybind11::list data_safety_;
    double Ds_;
    VehicleParams fw_params_;
    VehicleParams si_params_;
    std::map<size_t, std::pair<double, double>> minimums_;
};

} // namespace metrics
} // namespace barrier_certs
#endif // INCLUDE_BARRIER_CERTS_PLUGINS_METRICS_BARRIERCERTMETRICS_BARRIERCERTMETRICS_H_
