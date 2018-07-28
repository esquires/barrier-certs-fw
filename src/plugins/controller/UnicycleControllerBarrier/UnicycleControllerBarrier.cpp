#include <barrier-certs/plugins/controller/UnicycleControllerBarrier/UnicycleControllerBarrier.h>

#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/pubsub/Publisher.h>

#include <iostream>
#include <iomanip>

#include <boost/algorithm/clamp.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/map.hpp>

namespace sc = scrimmage;
namespace ba = boost::adaptors;
namespace br = boost::range;

REGISTER_PLUGIN(scrimmage::Controller,
                barrier_certs::controller::UnicycleControllerBarrier,
                UnicycleControllerBarrier_plugin)

namespace barrier_certs {
namespace controller {

void UnicycleControllerBarrier::init(std::map<std::string, std::string> &params) {
    id_ = parent_->id().id();
    controls_io_.init(params, vars_);

    // drawing variables
    circle_ = std::make_shared<scrimmage_proto::Shape>();
    sphere_ = std::make_shared<scrimmage_proto::Shape>();
    draw_dist_ = sc::str2bool(params.at("draw_dist"));
    draw_paths_ = sc::str2bool(params.at("draw_paths"));
    point_cloud_ = std::make_shared<scrimmage_proto::Shape>();
    draw_sensor_range_ = sc::str2bool(params.at("draw_sensor_range"));
    drawing_ids_ = sc::str2container<std::set<int>>(params.at("drawing_ids"), ",");

    centralized_ = sc::str2bool(params.at("centralized"));
    if (!centralized_) {
        bc_.init(params, parent_->id().id(), parent_->contacts(), parent_->rtree(), time_);
    }

    // logging variables
    log_delay_.delay = std::stod(params.at("log_delay"));
    pub_control_data_ = advertise("GlobalNetwork", "BarrierCertControls");
    pub_safety_data_ = advertise("GlobalNetwork", "BarrierCertSafety");
}

std::vector<double> UnicycleControllerBarrier::calc_u_hat() {
    auto u_hat = controls_io_.get_u_hat(vars_);

    if (!centralized_) {
        bc_.set_u_hat(id_, u_hat);
    }

    return u_hat;
}

bool UnicycleControllerBarrier::step(double t, double dt) {

    calc_u_hat();
    if (!centralized_) {
        bc_.safe_ctrl_input();
        u_safe_ = bc_.get_ctrl(id_);
        publish_data(bc_);
    }

    draw();

    const double v = u_safe_[0];
    const double w = u_safe_[1];
    const double dz = controls_io_.is_3d() && u_safe_.size() >= 2 ? u_safe_[2] : 0;

    controls_io_.set_outputs(v, w, dz, vars_);

    // if (!centralized_) {
    //     publish_data(bc_);
    // }
    return true;
}

void UnicycleControllerBarrier::set_u_safe(const std::vector<double> &u_safe) {
    u_safe_ = u_safe;
    if (controls_io_.get_clamp_actuator()) {
        controls_io_.clamp_actuator(u_safe_);
    }
}

void UnicycleControllerBarrier::publish_data(BarrierCert &bc) {
    bool publish = log_delay_.update(time_->t()).first;
    calc_safety_data(publish, bc);
    if (publish && time_->t() > 0) publish_controls_data(bc);
}

void UnicycleControllerBarrier::draw() {
    if (draw_paths_) {
        draw_path(bc_.paths(), point_cloud_);
        point_cloud_->set_persistent(false);
        point_cloud_->set_ttl(1);
        draw_shape(point_cloud_);
    }

    if (draw_dist_ && id_ == 1) {
        std::vector<int> other_ids;
        if (parent_->contacts()->size() == 20) {
            other_ids.push_back(2);
            other_ids.push_back(19);
            other_ids.push_back(20);
        } else {
            other_ids.push_back(2);
        }

        if (lines_.size() < other_ids.size()) {
            for (size_t i = 0; i < other_ids.size(); i++) {
                lines_.push_back(std::make_shared<scrimmage_proto::Shape>());
                texts_.push_back(std::make_shared<scrimmage_proto::Shape>());
            }
        }

        double mn_dist = std::numeric_limits<double>::infinity();
        for (int id : other_ids) {
            double dist =
                (parent_->contacts()->at(id).state()->pos() -
                 state_->pos()).norm();
            mn_dist = std::min(mn_dist, dist);
        }

        double scale = std::min(7.0, std::max(1.0, 0.1 * mn_dist));

        for (size_t i = 0; i < other_ids.size(); i++) {
            auto &p1 = state_->pos();
            auto &p2 = parent_->contacts()->at(other_ids[i]).state()->pos();
            std::stringstream ss;
            ss << std::fixed << std::setprecision(0) << (p1 - p2).norm() << "m";
            auto line = lines_[i];
            auto text = texts_[i];
            draw_dist(p1, p2, true, ss.str(), text, line, scale);
            draw_shape(line);
            draw_shape(text);
        }
    }

    if (draw_sensor_range_ && bc_.limited_sensing().sensing_range() > 0 &&
        drawing_ids_.count(id_)) {

        // circle_->set_opacity(0.25);
        // circle_->set_persistent(true);
        // sc::set(circle_->mutable_circle()->mutable_center(), state_->pos());
        // circle_->mutable_circle()->set_radius(bc_.limited_sensing().sensing_range());
        // sc::set(circle_->mutable_color(), 0, 0, 0);
        // draw_shape(circle_);
        sphere_->set_opacity(0.1);
        sphere_->set_persistent(true);
        sc::set(sphere_->mutable_sphere()->mutable_center(), state_->pos());
        sphere_->mutable_sphere()->set_radius(bc_.limited_sensing().sensing_range());
        sc::set(sphere_->mutable_color(), 0, 0, 0);
        draw_shape(sphere_);

    }
}

std::pair<double, size_t> UnicycleControllerBarrier::get_min_h(const std::map<int, BarrierCert::Data> &data) {
    if (data.empty()) {
        return {std::numeric_limits<double>::infinity(), 0};
    } else {
        auto cmp = [&](auto &kv1, auto &kv2) {return kv1.second.h < kv2.second.h;};
        auto it = br::min_element(data, cmp);
        return {it->second.h, it->first};
    }
}

void UnicycleControllerBarrier::calc_safety_data(bool publish, BarrierCert &bc) {
    size_t dist_id, h_id;
    double min_d, min_h;
    std::tie(min_d, dist_id) = min_dist(
        id_, state_->pos(), parent_->rtree(), parent_->contacts());
    std::tie(min_h, h_id) = get_min_h(bc.cached_data().at(id_));

    if (min_h < 0) {
        std::cout << time_->t()
            << ": h = " << min_h
            << " (" << id_ << ", " << h_id << ")" << std::endl;
    }

    if (publish && time_->t() > 0) {
        auto msg = std::make_shared<sc::Message<SafetyData>>();
        msg->data = {time_->t(), id_, min_d, dist_id, min_h, h_id};
        pub_safety_data_->publish(msg);
    }
}

void UnicycleControllerBarrier::publish_controls_data(BarrierCert &bc) {
    auto msg = std::make_shared<sc::Message<ControlsData>>();
    auto u_hat = calc_u_hat();
    auto u_safe_copy = u_safe_;

    msg->data = {time_->t(), id_, u_hat, u_safe_copy};
    pub_control_data_->publish(msg);
}
} // namespace controller
} // namespace barrier_certs
