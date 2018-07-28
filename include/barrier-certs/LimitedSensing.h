#ifndef INCLUDE_BARRIER_CERTS_LIMITEDSENSING_H_
#define INCLUDE_BARRIER_CERTS_LIMITEDSENSING_H_

#include <barrier-certs/HelperTypes.h>

#include <map>
#include <string>

namespace barrier_certs {

class LimitedSensing {
 public:
    LimitedSensing() = default;
    explicit LimitedSensing(std::map<std::string, std::string> &params);

    double tilde_h(double h);
    double psi(double xi, double val);
    double sensing_range() {return sensing_range_;}

    void set_beta(double beta) {beta_ = beta;}

 protected:

    double calc_xi();

    double Ds_ = 0.5;
    double beta_ = 0.5;
    double sensing_range_ = 1;
    double vel_fw_ = 1;
    double omega_fw_ = 1;
    double delta_ = 0;
};

} // namespace barrier_certs

#endif // INCLUDE_BARRIER_CERTS_LIMITEDSENSING_H_
