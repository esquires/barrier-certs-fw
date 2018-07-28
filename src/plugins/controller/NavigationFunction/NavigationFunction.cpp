#include <barrier-certs/plugins/controller/NavigationFunction/NavigationFunction.h>

#include <scrimmage/common/Time.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/common/RTree.h>

#include <barrier-certs/plugins/autonomy/Wpt/Wpt.h>
#include <barrier-certs/HelperTypes.h>
#include <barrier-certs/Utilities.h>

#include <iostream>
#include <limits>

#include <boost/algorithm/clamp.hpp>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                barrier_certs::controller::NavigationFunction,
                NavigationFunction_plugin)

namespace barrier_certs {
namespace controller {

void NavigationFunction::init(std::map<std::string, std::string> &params) {
    using Type = sc::VariableIO::Type;
    using Dir = sc::VariableIO::Direction;

    x_idx_in_ = vars_.declare(Type::position_x, Dir::In);
    y_idx_in_ = vars_.declare(Type::position_y, Dir::In);
    z_idx_in_ = vars_.declare(Type::position_z, Dir::In);

    turn_rate_idx_out_ = vars_.declare(Type::turn_rate, Dir::Out);

    auto get = [&](auto key){return std::stod(params.at(key));};
    speed_ = get("speed");
    vars_.output(vars_.declare(Type::speed, Dir::Out), speed_);
    vars_.output(vars_.declare(Type::velocity_z, Dir::Out), 0);

    rp_ = get("rp");
    k_ = get("k");
    Kp_ = get("Kp");
    Ks_ = get("Ks");
    delta_j_ = get("delta_j");
    delta_c_ = get("delta_c");
    rho_j_ = get("rho_j");
    rho_c_ = get("rho_c");
    K_ = get("K");
    theta_cutoff_ = get("theta_cutoff");
    rss_ = get("rss");
    turn_rate_max_ = sc::Angles::deg2rad(get("turn_rate_max"));

    add_swirling_ = sc::str2bool(params.at("add_swirling"));

    // logging variables
    pub_control_data_ = advertise("GlobalNetwork", "BarrierCertControls");
    pub_safety_data_ = advertise("GlobalNetwork", "BarrierCertSafety");
}

void NavigationFunction::calc_safety_data() {
    size_t id = parent_->id().id();

    size_t dist_id;
    double min_d;
    std::tie(min_d, dist_id) = min_dist(
        id, state_->pos(), parent_->rtree(), parent_->contacts());
    if (time_->t() > 0) {
        auto msg = std::make_shared<sc::Message<SafetyData>>();

        // dummy data since this isn't relevant for navigation functions
        double min_h = 1;
        size_t h_id = 1;

        msg->data = {time_->t(), id, min_d, dist_id, min_h, h_id};
        pub_safety_data_->publish(msg);
    }
}

bool NavigationFunction::step(double t, double dt) {
    is_dx = false;

    if (qo_.empty()) {
        for (auto &kv : *parent_->contacts()) {
            int i = kv.second.id().id();
            qo_[i] = get_pos(i);
        }
    }

    const double psi_i = parent_->state()->quat().yaw();

    auto[v_grad, v_grad2] = get_v_i_gradients();
    if (add_swirling_) {
      v_grad -= get_swirling_term(v_grad);
    }
    const double psi_d_i = std::atan2(-v_grad(1), -v_grad(0));
    const double dot_psi_d_i = std::atan2(-v_grad2(1), -v_grad2(0));

    // double turn_rate = K_ * (psi_d_i - psi_i) + dot_psi_d_i;
    double turn_rate = K_ * (sc::Angles::angle_diff_rad(psi_d_i, psi_i));
    turn_rate = boost::algorithm::clamp(turn_rate, -turn_rate_max_, turn_rate_max_);
    vars_.output(turn_rate_idx_out_, turn_rate);

    calc_safety_data();

    auto msg = std::make_shared<sc::Message<ControlsData>>();
    std::vector<double> u_hat {speed_, turn_rate, 0};
    size_t id = parent_->id().id();
    msg->data = {time_->t(), id, u_hat, u_hat};

    if (time_->t() > 0.0) {
      pub_control_data_->publish(msg);
    }

    return true;
}

double NavigationFunction::get_beta(double r_ij) {
    if (r_ij < 0) {
        return 0;
    } else if (r_ij < 1) {
        return f(r_ij);
    } else {
        return 1;
    }
}

double NavigationFunction::get_gamma_i() {
    Eigen::Vector2d qd_i = std::dynamic_pointer_cast<Wpt>(
        parent_->autonomies()[0])->wpt().head<2>();

    int i = parent_->id().id();
    Eigen::Vector2d qi = get_pos(i);
    const double dist2goal = (qi - qd_i).norm();

    const double dist2goal_squared = pow(dist2goal, 2);
    const double beta = get_beta(dist2goal / rp_);
    const double d_i = calc_d_i(i);
    return dist2goal_squared + Kp_ * beta * pow(d_i, 2);
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d> NavigationFunction::get_v_i_gradients() {

    const double v = get_V_i();
    int i = parent_->id().id();
    auto state = parent_->contacts()->at(i).state();
    Eigen::Vector3d orig_pos = state->pos();  // copy

    auto get_offset = [&](int pos_idx, double offset) {
        state->pos()(pos_idx) += offset;
        const double v_offset = get_V_i();
        state->pos() = orig_pos;
        return v_offset;
    };

    auto get_grad = [&](int pos_idx) {
        const double offset = 1;
        const double v_neg = get_offset(pos_idx, -offset);
        const double v_pos = get_offset(pos_idx, offset);
        const double dv = (v_pos - v_neg) / (2 * offset);
        const double dv2 = (v_pos - 2 * v + v_neg) / pow(offset, 2);
        return std::make_tuple(dv, dv2);
    };

    is_dx = true;
    auto[dvdx, dv2dx2] = get_grad(0);
    auto[dvdy, dv2dy2] = get_grad(1);
    is_dx = false;

    return std::make_tuple(Eigen::Vector2d(dvdx, dvdy), Eigen::Vector2d(dv2dx2, dv2dy2));
}


double NavigationFunction::get_V_i() {
    const double gamma_i = get_gamma_i();
    const double beta_i = get_beta_i();
    if (parent_->id().id() == 2 && !is_dx) {
      // std::cout << "beta_i = " << beta_i << std::endl;
    }
    return gamma_i / pow(pow(gamma_i, k_) + beta_i, 1 / k_);
}

std::vector<int> NavigationFunction::get_neigh() {
    double i = parent_->id().id();
    std::vector<sc::ID> neigh;
    parent_->rtree()->neighbors_in_range(
        parent_->state()->pos_const(), neigh, rss_, i);

    std::vector<int> out;
    out.reserve(neigh.size());
    std::transform(neigh.begin(), neigh.end(), std::back_inserter(out),
        [&](auto &id){return id.id();});
    return out;
}

double NavigationFunction::get_beta_i() {
    // just do fully connected network
    // so rss = infinity
    double out = 1;
    double i = parent_->id().id();
    for (int j: get_neigh()) {
        double r_ij = calc_r_ij(i, j);
        double r_ic = calc_r_ic(i);
        out *= get_beta(r_ij) * get_beta(r_ic);
    }

    return out;
}

double NavigationFunction::calc_d_i(int i) {
    Eigen::Vector2d qd_i = std::dynamic_pointer_cast<Wpt>(
        parent_->autonomies()[0])->wpt().head<2>();
    Eigen::Vector2d qi = get_pos(i);
    Eigen::Vector2d qo_i = qo_.at(i);

    Eigen::Vector2d beg2cur = qi - qo_i;
    Eigen::Vector2d beg2end_unit = (qd_i - qo_i).normalized();

    Eigen::Vector2d proj_on_nominal_path = (beg2cur.dot(beg2end_unit)) * beg2end_unit;

    Eigen::Vector2d vec_to_nominal_path = beg2cur - proj_on_nominal_path;

    double dist_to_nominal_path = vec_to_nominal_path.norm();

    return dist_to_nominal_path;
}

double NavigationFunction::f(double r_ij) {
    // paper describes it as something that moves between 0 and 1 as
    // r_ij moves between 0 and 1 so just do a parabola
    if (r_ij < 0) {
        return 0;
    } else if (r_ij < 0.5) {
        return pow(r_ij, 2);
    } else if (r_ij < 1.0) {
        return 1 - pow(1 - r_ij, 2);
    } else {
        return 1;
    }
}

Eigen::Vector2d NavigationFunction::get_pos(int i) {
    return parent_->contacts()->at(i).state()->pos().head<2>();
}

double NavigationFunction::calc_r_ij(int i, int j) {
    double dist = (get_pos(i) - get_pos(j)).norm();
    return (dist - rho_j_) / delta_j_;
}

double NavigationFunction::calc_r_ic(int i) {
    Eigen::Vector2d qc(0, 0);
    Eigen::Vector2d qi = get_pos(i);
    return (rho_c_ - (qc - qi).norm()) / delta_c_;
}

Eigen::Vector2d NavigationFunction::get_swirling_term(Eigen::Vector2d &grad_v) {
    int i = parent_->id().id();
    const Eigen::Vector2d pos_i = get_pos(i);
    const double norm_grad_v = grad_v.norm();

    Eigen::Vector2d out = Eigen::Vector2d::Zero();

    for (int &j : get_neigh()) {
        const Eigen::Vector2d pos_j = get_pos(j);
        const Eigen::Vector2d pos_diff = pos_i - pos_j;
        const double dist = pos_diff.norm();
        const double beta_i = get_beta((rss_ - dist) / rss_);

        Eigen::Vector3d pos_diff_3d {pos_diff(0), pos_diff(1), 0};
        Eigen::Vector3d vss_vec = pos_diff_3d.cross(Eigen::Vector3d::UnitZ());
        vss_vec /= pos_diff_3d.norm();
        out += (Ks_ * beta_i * norm_grad_v) * vss_vec.head<2>();

    }
    if (parent_->id().id() == 2 && !is_dx) {
      // std::cout << "swirling term: " << out(0) << ", " << out(1) << std::endl;
    }
    return out;
}

} // namespace controller
} // namespace barrier_certs

