#ifndef INCLUDE_BARRIER_CERTS_PLUGINS_CONTROLLER_UNICYCLECONTROLLERBARRIER_UNICYCLECONTROLLERBARRIER_H_
#define INCLUDE_BARRIER_CERTS_PLUGINS_CONTROLLER_UNICYCLECONTROLLERBARRIER_UNICYCLECONTROLLERBARRIER_H_

#include <barrier-certs/ControlsIO.h>
#include <barrier-certs/BarrierCert.h>

#include <scrimmage/common/DelayedTask.h>
#include <scrimmage/motion/Controller.h>

#include <utility>
#include <limits>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <set>

namespace barrier_certs {
namespace controller {

class UnicycleControllerBarrier : public scrimmage::Controller {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step(double /*t*/, double dt) override;
    std::vector<double> calc_u_hat();
    void set_u_safe(const std::vector<double> &u_safe);
    void publish_data(BarrierCert &bc);

 protected:
    ControlsIO controls_io_;

    void calc_safety_data(bool publish, BarrierCert &bc);
    void publish_controls_data(BarrierCert &bc);
    void draw();
    std::pair<double, size_t> get_min_h(const std::map<int, BarrierCert::Data> &data);

    // drawing variables
    std::set<int> drawing_ids_;
    bool draw_sensor_range_;
    bool draw_dist_;
    bool draw_paths_;
    std::vector<std::shared_ptr<scrimmage_proto::Shape>> lines_;
    std::vector<std::shared_ptr<scrimmage_proto::Shape>> texts_;
    std::shared_ptr<scrimmage_proto::Shape> circle_;
    std::shared_ptr<scrimmage_proto::Shape> sphere_;
    std::shared_ptr<scrimmage_proto::Shape> point_cloud_;

    // logging variables
    scrimmage::DelayedTask log_delay_;
    scrimmage::PublisherPtr pub_control_data_;
    scrimmage::PublisherPtr pub_safety_data_;

    // calculation of barrier variables
    size_t id_;
    bool centralized_;
    std::vector<double> u_safe_;
    BarrierCert bc_;
};

} // namespace controller
} // namespace barrier_certs
#endif // INCLUDE_BARRIER_CERTS_PLUGINS_CONTROLLER_UNICYCLECONTROLLERBARRIER_UNICYCLECONTROLLERBARRIER_H_
