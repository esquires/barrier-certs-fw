#ifndef INCLUDE_BARRIER_CERTS_PLUGINS_INTERACTION_BARRIERCERTCENTRALIZED_BARRIERCERTCENTRALIZED_H_
#define INCLUDE_BARRIER_CERTS_PLUGINS_INTERACTION_BARRIERCERTCENTRALIZED_BARRIERCERTCENTRALIZED_H_

#include <scrimmage/simcontrol/EntityInteraction.h>

#include <barrier-certs/BarrierCert.h>

#include <map>
#include <list>
#include <string>

namespace scrimmage {
class Entity;
using EntityPtr = std::shared_ptr<Entity>;
}

namespace barrier_certs {

namespace controller {
class UnicycleControllerBarrier;
} // namespace controller

namespace interaction {

class BarrierCertCentralized : public scrimmage::EntityInteraction {
 public:
    bool init(std::map<std::string, std::string> &mission_params,
              std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                 double t, double dt) override;
 protected:
    BarrierCert bc_;
    bool centralized_;
    std::map<int, std::shared_ptr<controller::UnicycleControllerBarrier>> bc_ctrl_;
    bool draw_h_ = false;
    std::shared_ptr<scrimmage_proto::Shape> point_cloud_;
    std::shared_ptr<scrimmage_proto::Shape> line_;
    std::shared_ptr<scrimmage_proto::Shape> text_;
    std::map<std::string, std::string> plugin_params_;
    bool draw_paths_;
};

} // namespace interaction
} // namespace barrier_certs
#endif // INCLUDE_BARRIER_CERTS_PLUGINS_INTERACTION_BARRIERCERTCENTRALIZED_BARRIERCERTCENTRALIZED_H_
