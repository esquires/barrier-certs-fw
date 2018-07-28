
#ifndef INCLUDE_BARRIER_CERTS_UTILITIES_H_
#define INCLUDE_BARRIER_CERTS_UTILITIES_H_

#include <barrier-certs/HelperTypes.h>
#include <scrimmage/proto/Shape.pb.h>

#include <Eigen/Dense>

#include <memory>
#include <vector>
#include <utility>
#include <unordered_map>
#include <map>
#include <string>
#include <tuple>

namespace scrimmage_proto {
class Shape;
}

namespace scrimmage {
using ShapePtr = std::shared_ptr<scrimmage_proto::Shape>;

class RTree;
using RTreePtr = std::shared_ptr<scrimmage::RTree>;

class Contact;
using ContactMap = std::unordered_map<int, Contact>;
using ContactMapPtr = std::shared_ptr<ContactMap>;
}

namespace barrier_certs {

void draw_dist(const Eigen::Vector3d &p1,
               const Eigen::Vector3d &p2,
               bool set_text_loc,
               const std::string &text,
               scrimmage::ShapePtr &text_shape,
               scrimmage::ShapePtr &line,
               double text_scale);

void draw_path(
        const std::vector<std::vector<scrimmage_proto::Vector3d>> &paths,
        scrimmage::ShapePtr &point_cloud);

std::pair<double, int> min_dist(
    int id1, const Eigen::Vector3d &p1,
    scrimmage::RTreePtr rtree, scrimmage::ContactMapPtr contacts);

double dist_sq_fw_fw_straight(
    const std::vector<double> &x,
    double v1, double v2);

double dist_sq_fw_fw_turn(
    const std::vector<double> &x,
    double v, double w, double sigma,
    double delta, bool first_is_lower_id);

std::pair<CalcType, VehicleParams> parse_fw_params(
    const std::map<std::string, std::string> &params);

template <class HFunc>
double calc_ph_px(double h, std::vector<double> &x, int idx, HFunc calc_h) {
    const double eps = 0.0001;
    x[idx] += eps;
    double h_offset = calc_h(x);
    double out = (h_offset - h) / eps;
    x[idx] -= eps;
    return out;
}

double adjust_gamma_straight_vel(
    size_t id, double straight_pct_offset, double v, double vmin, double vmax);

std::tuple<double, double, double, double, double, double>
parse_fw_gamma(CalcType calc_type,
               size_t id1, size_t id2,
               double vel_min,
               scrimmage::ContactMapPtr contacts,
               const std::vector<double> &gamma);

std::tuple<double, double, double, double, double, double, double, double>
parse_fw_state(const std::vector<double> &x);

double bank_to_turn_rate(double vel, double bank);
double turn_rate_to_bank(double vel, double turn_rate);
} // namespace barrier_certs

#endif // INCLUDE_BARRIER_CERTS_UTILITIES_H_
