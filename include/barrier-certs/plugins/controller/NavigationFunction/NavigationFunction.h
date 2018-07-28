#ifndef INCLUDE_BARRIER_CERTS_PLUGINS_CONTROLLER_NAVIGATIONFUNCTION_NAVIGATIONFUNCTION_H_
#define INCLUDE_BARRIER_CERTS_PLUGINS_CONTROLLER_NAVIGATIONFUNCTION_NAVIGATIONFUNCTION_H_

#include <scrimmage/motion/Controller.h>

#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace barrier_certs {
namespace controller {

class NavigationFunction : public scrimmage::Controller {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;

 protected:
    bool is_dx = false;
    // controller input/output vars
    uint8_t x_idx_in_ = 0;
    uint8_t y_idx_in_ = 0;
    uint8_t z_idx_in_ = 0;
    uint8_t turn_rate_idx_out_ = 0;
    double turn_rate_max_ = 0;
    double speed_ = 0;

    // navigation function vars
    double rp_;  // distance to goal when it is considered achieved
    double k_;  // tuning for value function
    double Kp_; // tuning in gamma func for weighting goal vs collisions
    double Ks_; // tuning for swirling parameter
    double delta_j_;  // rho + delta is the conflict distance
    double delta_c_;  // tuning for center of configuration space
    double rho_j_;  // collision distance
    double rho_c_;  // configuration space radius
    double K_;  // tuning parameter for final controller
    double theta_cutoff_;  // swirling parameter
    double rss_;  // sensing distance
    bool add_swirling_;  // whether to include the section IIIB term in the value function

    std::map<int, Eigen::Vector2d> qo_;  // starting positions (origin)

    // navigation function functions
    std::tuple<double, double> nominal_controller();

    std::tuple<Eigen::Vector2d, Eigen::Vector2d> get_v_i_gradients();
    double get_V_i();
    double get_gamma_i();
    double f(double r_ij);
    double get_beta(double r_ij);
    double get_beta_i();
    double calc_d_i(int i);

    double calc_r_ij(int i, int j);
    double calc_r_ic(int i);

    Eigen::Vector2d get_pos(int i);
    Eigen::Vector2d get_swirling_term(Eigen::Vector2d &grad_v);
    std::vector<int> get_neigh();

    // publishers
    scrimmage::PublisherPtr pub_control_data_;
    scrimmage::PublisherPtr pub_safety_data_;
    void calc_safety_data();

};
} // namespace controller
} // namespace barrier_certs
#endif // INCLUDE_BARRIER_CERTS_PLUGINS_CONTROLLER_NAVIGATIONFUNCTION_NAVIGATIONFUNCTION_H_
