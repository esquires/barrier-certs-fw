#include <barrier-certs/plugins/interaction/BarrierCertCentralized/BarrierCertCentralized.h>
#include <barrier-certs/plugins/controller/UnicycleControllerBarrier/UnicycleControllerBarrier.h>
#include <barrier-certs/Utilities.h>

#include <scrimmage/common/Time.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>

#include <memory>
#include <limits>
#include <iostream>
#include <iomanip>

#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/min_element.hpp>

namespace sc = scrimmage;
namespace ba = boost::adaptors;
namespace br = boost::range;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                barrier_certs::interaction::BarrierCertCentralized,
                BarrierCertCentralized_plugin)

namespace barrier_certs {
namespace interaction {

bool BarrierCertCentralized::init(std::map<std::string, std::string> &/*mission_params*/,
                                  std::map<std::string, std::string> &plugin_params) {
    point_cloud_ = std::make_shared<scrimmage_proto::Shape>();
    line_ = std::make_shared<scrimmage_proto::Shape>();
    text_ = std::make_shared<scrimmage_proto::Shape>();
    draw_h_ = sc::str2bool(plugin_params.at("draw_h"));
    draw_paths_ = sc::str2bool(plugin_params.at("draw_paths"));
    centralized_ = sc::str2bool(plugin_params.at("centralized"));
    plugin_params_ = plugin_params;
    return true;
}


bool BarrierCertCentralized::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double /*t*/, double /*dt*/) {
    if (!centralized_) return true;
    if (ents.empty()) return false;

    if (time_->t() <= 0) {
        if (centralized_) {
            // std::cout << "calling bc_.init" << std::endl;
            bc_.init(plugin_params_, parent_->id().id(), parent_->contacts(), parent_->rtree(), time_);
        }

        for (auto &e : ents) {
            auto &ctrls = e->controllers();
            if (ctrls.size() < 2) continue;

            auto nominal_controller = *ctrls.begin();
            // nominal_controller->step(time_->t(), time_->dt()); // nominal controller

            auto bc_controller = *(ctrls.begin() + 1);
            auto ctrl = std::dynamic_pointer_cast<controller::UnicycleControllerBarrier>(bc_controller);
            int id = e->id().id();

            if (!ctrl) {
                std::cout << "could not cast " <<
                    id << " controller to UnicycleControllerBarrier" << std::endl;
                return false;
            }

            bc_ctrl_[id] = ctrl;
        }
    }

    auto ids_sorted = *parent_->contacts() | ba::map_keys;
    std::set<int> ids(ids_sorted.begin(), ids_sorted.end());

    for (auto &kv : bc_ctrl_) {
        bc_.set_u_hat(kv.first, kv.second->calc_u_hat());
    }

    bc_.safe_ctrl_input();
    // std::cout << time_->t() << ": ";

    for (auto &kv : bc_ctrl_) {
        int id = kv.first;
        auto &ctrl = kv.second;

        auto u_safe = bc_.get_ctrl(id);
        ctrl->set_u_safe(u_safe);
        ctrl->publish_data(bc_);
    }

    if (draw_paths_) {
        draw_path(bc_.paths(), point_cloud_);
        draw_shape(point_cloud_);
    }

    if (draw_h_) {
        Eigen::Vector3d eig_p1, eig_p2;
        auto &paths = bc_.paths();
        if (draw_paths_ && paths.size() > 1) {
            // find the point along the paths that is minimal and draw a line
            // between those two points
            size_t mn = std::min(paths[0].size(), paths[1].size());
            std::vector<double> distances;
            auto get_dist = [&](auto &p1, auto &p2) {
                return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
            };
            std::transform(paths[0].begin(), paths[0].begin() + mn,
                           paths[1].begin(), std::back_inserter(distances), get_dist);
            auto it = br::min_element(distances);
            size_t idx = std::distance(distances.begin(), it);
            sc::set(eig_p1, paths[0][idx]);
            sc::set(eig_p2, paths[1][idx]);

        } else {
            // draw a line between the vehicles
            eig_p1 = parent_->contacts()->at(1).state()->pos();
            eig_p2 = parent_->contacts()->at(2).state()->pos();
        }

        std::stringstream ss;
        ss << "h = " << std::fixed << std::setprecision(0) << bc_.cached_data()[1][2].h;

        draw_dist(eig_p1, eig_p2, true, ss.str(), text_, line_, 10);
        draw_shape(line_);
        draw_shape(text_);
    }
    return true;
}
} // namespace interaction
} // namespace barrier_certs
