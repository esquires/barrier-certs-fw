#include <barrier-certs/plugins/autonomy/Wpt/Wpt.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/pubsub/Publisher.h>

#include <iostream>

#include <boost/algorithm/string.hpp>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, barrier_certs::Wpt, Wpt_plugin)

namespace barrier_certs {

void Wpt::init(std::map<std::string, std::string> &params) {

    using Type = sc::VariableIO::Type;
    using Dir = sc::VariableIO::Direction;

    const uint8_t x_idx_out = vars_.declare(Type::position_x, Dir::Out);
    const uint8_t y_idx_out = vars_.declare(Type::position_y, Dir::Out);
    const uint8_t z_idx_out = vars_.declare(Type::position_z, Dir::Out);
    const uint8_t alt_idx = vars_.declare(Type::desired_altitude, Dir::Out);

    speed_out_idx_ = vars_.declare(Type::desired_speed, Dir::Out);
    heading_out_idx_ = vars_.declare(Type::desired_heading, Dir::Out);
    dalt_out_idx_ = vars_.declare(Type::velocity_z, Dir::Out);

    exit_on_reaching_wpt_ = sc::str2bool(params.at("exit_on_reaching_wpt"));
    stop_thresh_ = std::stod(params.at("stop_thresh"));
    std::vector<double> wpt_vec = sc::str2container<std::vector<double>>(params.at("wpt"), ",");

    if (wpt_vec.size() != 3) {
        std::cout << "wpt_vec is the wrong size" << std::endl;
        return;
    }

    wpt_ << wpt_vec[0], wpt_vec[1], wpt_vec[2];
    // std::cout << "wpt: " << wpt_vec[0] << ", " << wpt_vec[1] << ", " << wpt_vec[2] << std::endl;

    draw_wpt_ = sc::str2bool(params.at("draw_wpt"));
    // int id = parent_->id().id();
    // if (draw_wpt && (id == 3 || id == 9)) {
    if (draw_wpt_) {
        sphere_ = std::make_shared<scrimmage_proto::Shape>();
        sc::set(sphere_->mutable_sphere()->mutable_center(), wpt_(0), wpt_(1), 0);

        const double wpt_size = std::stod(params.at("wpt_size"));
        sphere_->mutable_sphere()->set_radius(wpt_size);
        sphere_->set_persistent(true);
        // const std::vector<int> light_blue {68, 106, 255};
        const std::vector<int> dark_green {0, 85, 0};
        sc::set(sphere_->mutable_color(), dark_green);
        // draw_shape(sphere_);

        // also draw straight path in red
        path_ = std::make_shared<scrimmage_proto::Shape>();
        auto &pos = state_->pos();
        auto ln = path_->mutable_line();

        auto pos2d = pos.head<2>();
        auto wpt2d = wpt_.head<2>();
        auto diff = wpt2d - pos2d;
        Eigen::Vector2d beg = pos2d + 0.1 * diff;
        sc::set(ln->mutable_start(), beg(0), beg(1), 0);

        auto wpt_end = pos2d + 0.95 * diff;
        sc::set(ln->mutable_end(), wpt_end(0), wpt_end(1), 0);
        ln->set_width(5);

        const std::vector<int> dark_red {150, 0, 0};
        sc::set(path_->mutable_color(), dark_red);

        // draw_shape(path_);

        // ends of arrows
        Eigen::Vector2d arrow_end = wpt_end + 0.05 * (Eigen::Rotation2D<double>(M_PI / 4).toRotationMatrix() * (-diff));
        arrow1_ = std::make_shared<scrimmage_proto::Shape>();
        ln = arrow1_->mutable_line();
        ln->set_width(5);
        sc::set(ln->mutable_start(), wpt_end(0), wpt_end(1), 0);
        sc::set(ln->mutable_end(), arrow_end(0), arrow_end(1), 0);
        sc::set(arrow1_->mutable_color(), dark_red);
        // draw_shape(arrow1_);

        arrow_end = wpt_end + 0.05 * (Eigen::Rotation2D<double>(-M_PI / 4).toRotationMatrix() * (-diff));
        arrow2_ = std::make_shared<scrimmage_proto::Shape>();
        ln = arrow2_->mutable_line();
        ln->set_width(5);
        sc::set(ln->mutable_start(), wpt_end(0), wpt_end(1), 0);
        sc::set(ln->mutable_end(), arrow_end(0), arrow_end(1), 0);
        sc::set(arrow2_->mutable_color(), dark_red);
        // draw_shape(arrow2_);
    }

    vars_.output(x_idx_out, wpt_(0));
    vars_.output(y_idx_out, wpt_(1));
    vars_.output(z_idx_out, wpt_(2));
    vars_.output(alt_idx, 0);

    pub_ = advertise("GlobalNetwork", "Wpt");
    gain_ = std::stod(params.at("gain"));
}

bool Wpt::step_autonomy(double t, double dt) {
    int step_num = t / dt;
    if (draw_wpt_ && step_num - parent_->id().id() == 1) {
      draw_shape(sphere_);
      draw_shape(path_);
      draw_shape(arrow1_);
      draw_shape(arrow2_);
    }
    const Eigen::Vector2d diff = wpt_.head<2>() - state_->pos().head<2>();
    const double dist_to_wpt = diff.norm();
    // if (parent_->id().id() == 1) {
    //     std::cout << t << ", dist_to_wpt = " << dist_to_wpt << std::endl;
    // }

    const double des_speed = gain_ * dist_to_wpt;
    const double des_heading = atan2(diff(1), diff(0));
    const double dalt = wpt_(2) - state_->pos()(2);

    vars_.output(speed_out_idx_, des_speed);
    vars_.output(heading_out_idx_, des_heading);
    vars_.output(dalt_out_idx_, gain_ * dalt);

    if (dist_to_wpt < stop_thresh_) {
        pub_->publish(std::make_shared<sc::Message<int>>(parent_->id().id()));
        if (exit_on_reaching_wpt_) return false;
    }
    return true;
}
} // namespace barrier_certs
