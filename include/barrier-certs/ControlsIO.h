
#ifndef INCLUDE_BARRIER_CERTS_CONTROLSIO_H_
#define INCLUDE_BARRIER_CERTS_CONTROLSIO_H_

#include <scrimmage/common/VariableIO.h>

#include <map>
#include <string>
#include <vector>

namespace barrier_certs {

class ControlsIO {
 public:
    void init(
        std::map<std::string, std::string> &params,
        scrimmage::VariableIO &vars);
    void set_outputs(double v, double w, double dz, scrimmage::VariableIO &vars);
    std::vector<double> get_u_hat(scrimmage::VariableIO &vars);
    void clamp_actuator(std::vector<double> &vec);

    bool is_3d() const {return is_3d_;}
    bool get_clamp_actuator() const {return clamp_actuators_;}

 protected:
    uint8_t speed_idx_in_ = 0;
    uint8_t turn_rate_idx_in_ = 0;
    uint8_t velocity_z_idx_in_ = 0;

    uint8_t speed_idx_out_ = 0;
    uint8_t des_speed_idx_out_ = 0;

    uint8_t turn_rate_idx_out_ = 0;
    uint8_t des_roll_idx_out_ = 0;

    uint8_t velocity_z_idx_out_ = 0;
    uint8_t alt_idx_out_ = 0;

    bool is_3d_;

    double vel_max_ = 0;
    double vel_min_ = 0;
    double turn_rate_max_ = 0;
    double velocity_z_max_ = 0;
    bool clamp_actuators_ = false;
};

} // namespace barrier_certs
#endif // INCLUDE_BARRIER_CERTS_CONTROLSIO_H_
