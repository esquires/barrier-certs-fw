#ifndef INCLUDE_BARRIER_CERTS_PLUGINS_AUTONOMY_WPT_WPT_H_
#define INCLUDE_BARRIER_CERTS_PLUGINS_AUTONOMY_WPT_WPT_H_

#include <scrimmage/autonomy/Autonomy.h>

#include <string>
#include <map>

namespace barrier_certs {
class Wpt : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;
    const Eigen::Vector3d wpt() {return wpt_;}

 protected:
    double stop_thresh_ = -1;
    bool exit_on_reaching_wpt_ = false;

    Eigen::Vector3d wpt_;

    uint8_t speed_out_idx_ = 0;
    uint8_t heading_out_idx_ = 0;
    uint8_t dalt_out_idx_ = 0;

    double gain_ = 1;
    bool draw_wpt_ = false;

    std::shared_ptr<scrimmage_proto::Shape> sphere_;
    std::shared_ptr<scrimmage_proto::Shape> path_;
    std::shared_ptr<scrimmage_proto::Shape> arrow1_;
    std::shared_ptr<scrimmage_proto::Shape> arrow2_;

    scrimmage::PublisherPtr pub_;
};
} // namespace barrier_certs
#endif // INCLUDE_BARRIER_CERTS_PLUGINS_AUTONOMY_WPT_WPT_H_
