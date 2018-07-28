#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>

#include <barrier-certs/ControlsIO.h>
#include <barrier-certs/Utilities.h>

#include <boost/algorithm/clamp.hpp>

namespace sc = scrimmage;

namespace barrier_certs {

void ControlsIO::init(
        std::map<std::string, std::string> &params,
        scrimmage::VariableIO &vars) {
    // Declare variables for controllers
    using Type = sc::VariableIO::Type;
    using Dir = sc::VariableIO::Direction;
    is_3d_ = sc::str2bool(params.at("is_3d"));

    vel_max_ = std::stod(params.at("vel_max"));
    vel_min_ = std::stod(params.at("vel_min"));
    turn_rate_max_ = sc::Angles::deg2rad(std::stod(params.at("turn_rate_max")));
    velocity_z_max_ = std::stod(params.at("velocity_z_max"));
    clamp_actuators_ = sc::str2bool(params.at("clamp_actuators"));

    speed_idx_in_ = vars.declare(Type::speed, Dir::In);
    turn_rate_idx_in_ = vars.declare(Type::turn_rate, Dir::In);
    if (is_3d_) {
        velocity_z_idx_in_ = vars.declare(Type::velocity_z, Dir::In);
    }

    speed_idx_out_ = vars.declare(Type::speed, Dir::Out);

    turn_rate_idx_out_ = vars.declare(Type::turn_rate, Dir::Out);
    // des_roll_idx_out_ = vars.declare(Type::desired_roll, Dir::Out);

    velocity_z_idx_out_ = vars.declare(Type::velocity_z, Dir::Out);

    // not considering this but makes unicycle work
    // uint8_t pitch_rate_idx_out = vars.declare(Type::pitch_rate, Dir::Out);
    // vars.output(pitch_rate_idx_out, 0);
}

void ControlsIO::set_outputs(double v, double w, double dz, scrimmage::VariableIO &vars) {
    // const double bank = turn_rate_to_bank(v, w);

    vars.output(speed_idx_out_, v);
    vars.output(des_speed_idx_out_, v);

    vars.output(turn_rate_idx_out_, w);

    // vars.output(des_roll_idx_out_, bank);
    vars.output(velocity_z_idx_out_, dz);
}

std::vector<double> ControlsIO::get_u_hat(scrimmage::VariableIO &vars) {
    std::vector<double> u_hat {vars.input(speed_idx_in_), vars.input(turn_rate_idx_in_)};
    if (is_3d_) {
        u_hat.push_back(vars.input(velocity_z_idx_in_));
    }

    if (clamp_actuators_) {
        clamp_actuator(u_hat);
    }
    return u_hat;
}

void ControlsIO::clamp_actuator(std::vector<double> &vec) {
    vec[0] = boost::algorithm::clamp(vec[0], vel_min_, vel_max_);
    vec[1] = boost::algorithm::clamp(vec[1], -turn_rate_max_, turn_rate_max_);
    if (is_3d_ && vec.size() >= 2) {
        vec[2] = boost::algorithm::clamp(vec[2], -velocity_z_max_, velocity_z_max_);
    }
}

} // namespace barrier_certs
