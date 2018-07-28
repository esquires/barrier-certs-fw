
#include <scrimmage/common/RTree.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>
#include <scrimmage/proto/Color.pb.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/parse/ParseUtils.h>

#include <barrier-certs/Utilities.h>

#include <iomanip>

#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/max_element.hpp>

namespace sp = scrimmage_proto;
namespace sc = scrimmage;
namespace ba = boost::adaptors;
namespace br = boost::range;

namespace barrier_certs {

void draw_dist(const Eigen::Vector3d &p1,
               const Eigen::Vector3d &p2,
               bool set_text_loc,
               const std::string &text,
               scrimmage::ShapePtr &text_shape,
               scrimmage::ShapePtr &line,
               double text_scale) {

    sp::Color clr;
    clr.set_r(255);
    clr.set_g(0);
    clr.set_b(0);

    // the line
    sc::set(line->mutable_color(), clr);
    line->set_opacity(0.75);
    line->set_persistent(true);
    line->set_ttl(1);
    sc::set(line->mutable_line()->mutable_start(), p1);
    sc::set(line->mutable_line()->mutable_end(), p2);

    // the text
    text_shape->set_persistent(true);
    text_shape->set_ttl(1);
    text_shape->set_opacity(1.0);
    sc::set(text_shape->mutable_color(), clr);

    Eigen::Vector3d diff = p2 - p1;
    text_shape->mutable_text()->set_scale(text_scale);

    if (set_text_loc) {
        Eigen::Vector3d text_loc = 1.15 * (p2 - p1) + p1;

        // const double offset = 3;
        // Eigen::Vector2d text_loc_2d = offset *
        //     (Eigen::Rotation2D<double>(M_PI / 2) * diff.head<2>().normalized());
        // Eigen::Vector3d text_loc(text_loc_2d(0), text_loc_2d(1), 0);

        sc::set(text_shape->mutable_text()->mutable_center(), text_loc);
    }

    text_shape->mutable_text()->set_text(text);
}

void draw_path(
        const std::vector<std::vector<scrimmage_proto::Vector3d>> &paths,
        scrimmage::ShapePtr &point_cloud) {
    point_cloud->mutable_pointcloud()->clear_point();
    point_cloud->mutable_pointcloud()->clear_color();
    for (auto &path : paths) {
        for (size_t i = 0; i < path.size(); i++) {
            auto *pt = point_cloud->mutable_pointcloud()->add_point();
            auto *clr = point_cloud->mutable_pointcloud()->add_color();
            sc::set(clr, 255, 0, 0);
            *pt = path[i];
        }
    }
    point_cloud->mutable_pointcloud()->set_size(3);
}

std::pair<double, int> min_dist(
        int id1, const Eigen::Vector3d &p1,
        scrimmage::RTreePtr rtree, scrimmage::ContactMapPtr contacts) {

    std::vector<sc::ID> neigh;
    rtree->nearest_n_neighbors(p1, neigh, 1, id1);
    double d = std::numeric_limits<double>::infinity();
    if (!neigh.empty()) {
        int id2 = neigh[0].id();
        auto &p2 = contacts->at(id2).state()->pos();
        d = (p1 - p2).norm();
        return {d, id2};
    } else {
        return {d, -1};
    }
}

std::tuple<double, double, double, double, double, double, double, double>
parse_fw_state(const std::vector<double> &x) {
    const bool is_3d = x.size() == 8;
    const double nan = std::numeric_limits<double>::quiet_NaN();

    return is_3d ?
        std::tuple(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]) :
        std::tuple(x[0], x[1], x[2], nan, x[3], x[4], x[5], nan);
}

std::tuple<double, double, double, double, double, double>
parse_fw_gamma(CalcType calc_type,
               size_t id1, size_t id2,
               double vel_min,
               scrimmage::ContactMapPtr contacts,
               const std::vector<double> &gamma) {
    const bool is_3d = gamma.size() == 6;
    const double nan = std::numeric_limits<double>::quiet_NaN();

    const double v1 = gamma[0];
    const double w1 = gamma[1];
    const double v2 = gamma[is_3d ? 3 : 2];
    const double w2 = gamma[is_3d ? 4 : 3];
    const double dz1 = is_3d ? gamma[2] : nan;
    const double dz2 = is_3d ? gamma[5] : nan;

    return std::tuple(v1, w1, dz1, v2, w2, dz2);
}

double dist_sq_fw_fw_turn(
        const std::vector<double> &x,
        double v, double w, double sigma,
        double delta, bool first_is_lower_id) {

    double x10, y10, th10, z10, x20, y20, th20, z20;
    std::tie(x10, y10, th10, z10, x20, y20, th20, z20) =
        parse_fw_state(x);
    const bool is_3d = !std::isnan(z10);
    const double th = first_is_lower_id ? th10 : th20;

    const double r = v / w;
    const double b10 = x10 - sigma * r * sin(th10);
    const double b20 = x20 - r * sin(th20);
    const double db = b10 - b20;

    const double c10 = y10 + sigma * r * cos(th10);
    const double c20 = y20 + r * cos(th20);
    const double dc = c10 - c20;

    const double dz = is_3d ? z10 - z20 : 0;

    double a1 = pow(db, 2) + pow(dc, 2) + pow(dz, 2) +
        (1 + pow(sigma, 2)) * pow(r, 2) -
        2 * sigma * pow(r, 2) * cos(th10 - th20) -
        2 * delta;

    auto add = [&](auto func) {
        return 2 * r * (
            db * (sigma * func(th10 - M_PI / 2) - func(th20 - M_PI / 2)) +
            dc * (-sigma * func(th10) + func(th20))) +
            delta * func(th - M_PI / 2) - delta * func(th);
    };

    const double re = add(cos);
    const double im = add(sin);
    const double a2 = sqrt(pow(re, 2) + pow(im, 2));

    return a1 - a2;
}

double dist_sq_fw_fw_straight(
    const std::vector<double> &x,
    double v1, double v2) {

    double x10, y10, th10, z10, x20, y20, th20, z20;
    std::tie(x10, y10, th10, z10, x20, y20, th20, z20) =
        parse_fw_state(x);
    const bool is_3d = !std::isnan(z10);

    const double db = x10 - x20;
    const double dc = y10 - y20;
    const double dz = is_3d ? z10 - z20 : 0;

    const double C = v1 * cos(th10) - v2 * cos(th20);
    const double S = v1 * sin(th10) - v2 * sin(th20);

    const double c = pow(db, 2) + pow(dc, 2);
    const double b = 2 * (db * C + dc * S);
    const double a = pow(C, 2) + pow(S, 2);

    double min_t = std::max(0.0, -b / (2 * a));
    double dist_squared = c + b * min_t + a * pow(min_t, 2) + pow(dz, 2);
    return dist_squared;
}

std::pair<CalcType, VehicleParams> parse_fw_params(
        const std::map<std::string, std::string> &params) {
    const double vel_min = std::stod(params.at("vel_min"));
    const double vel_max = std::stod(params.at("vel_max"));
    const double turn_rate_max =
        sc::Angles::deg2rad(std::stod(params.at("turn_rate_max")));

    const std::string calc_type_str = params.at("calc_type");
    const double pct_offset = std::stod(params.at("pct_offset"));
    const double straight_pct_offset =
        std::stod(params.at("straight_pct_offset"));
    const bool is_3d = sc::str2bool(params.at("is_3d"));

    const double vmin = vel_min + pct_offset * (vel_max - vel_min);
    CalcType calc_type;
    VehicleParams fw_params;
    auto set_gamma = [&](double v1, double v2, double omega) {
        if (is_3d) {
            fw_params.gamma = {v1, omega, 0, v2, omega, 0};
        } else {
            fw_params.gamma = {v1, omega, v2, omega};
        }
    };

    if (calc_type_str == "straight") {
        calc_type = CalcType::STRAIGHT;
        const double v2 = vel_min + straight_pct_offset * (vel_max - vel_min);
        set_gamma(vmin, v2, 0);
    } else if (calc_type_str == "turn") {
        calc_type = CalcType::TURN;
        const double omega_max = (1 - pct_offset) * turn_rate_max;
        const double sigma = std::stod(params.at("sigma"));
        set_gamma(sigma * vmin, vmin, omega_max);
    }

    fw_params.lims = {{vel_min, vel_max}, {-turn_rate_max, turn_rate_max}};
    if (is_3d) {
        const double velocity_z_max = std::stod(params.at("velocity_z_max"));
        fw_params.lims.push_back({-velocity_z_max, velocity_z_max});
    }

    return {calc_type, fw_params};
}

double adjust_gamma_straight_vel(
        size_t id, double straight_pct_offset, double v, double vmin, double vmax) {
    const double v_adj = (1 + id * straight_pct_offset) * v;
    if (v_adj > vmax) {
        throw std::runtime_error("v_adj > vmax");
    } else if (v_adj < vmin) {
        throw std::runtime_error("v_adj < vmin");
    }
    return v_adj;
}

double bank_to_turn_rate(double vel, double bank) {
    return -9.81 * tan(bank) / vel;
}

double turn_rate_to_bank(double vel, double turn_rate) {
    return -atan2(vel * turn_rate, 9.81);
}

} // namespace barrier_certs
