#include <osqp.h>

#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Time.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <barrier-certs/BarrierCert.h>
#include <barrier-certs/Utilities.h>

#include <iostream>
#include <iomanip>

#include <boost/math/special_functions/sign.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/fill.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/range/numeric.hpp>

namespace sc = scrimmage;
namespace ba = boost::adaptors;
namespace br = boost::range;
using boost::accumulate;

namespace barrier_certs {

void BarrierCert::safe_ctrl_input() {

    const auto ids = get_ids();
    update_cached_data(ids);
    calc_paths(ids);

    const std::vector<size_t> sizes = u_hat_sizes();
    unsigned int size_u_hat = accumulate(sizes, 0);
    set_u_map(sizes);

    size_t num_bc = get_num_bc(ids);

    qp_.m = 2 * size_u_hat + num_bc;
    std::map<int, std::map<int, double>> A; // col: row, value

    qp_.l.resize(qp_.m);
    qp_.u.resize(qp_.m);

    set_qp_cost(size_u_hat);
    set_bc_constraints(ids, 0, A);
    set_actuator_constraints(ids, num_bc, A);
    set_qp_constraints(A);

    calc_qp();
}

void BarrierCert::print_qp() {
    std::ostream_iterator<double> it_d(std::cout, ", ");
    std::ostream_iterator<int> it_i(std::cout, ", ");

    auto print = [&](auto str, auto &v, auto it) {
        std::cout << str << ": ";
        br::copy(v, it);
        std::cout << std::endl;
    };
    std::cout << std::setprecision(3);
    std::cout << "id: " << id_ << std::endl;
    std::cout << "P_nnz = " << qp_.P_nnz << std::endl;
    print("P_x", qp_.P_x, it_d);
    print("P_i", qp_.P_i, it_i);
    print("P_p", qp_.P_p, it_i);
    print("q", qp_.q, it_d);

    std::cout << std::endl;

    print("l", qp_.l, it_d);
    print("u", qp_.u, it_d);

    std::cout << std::endl;
    std::cout << "A_nnz = " << qp_.A_nnz << std::endl;
    print("A_x", qp_.A_x, it_d);
    print("A_i", qp_.A_i, it_i);
    print("A_p", qp_.A_p, it_i);
}

void print_csc_matrix_foo(csc *M, const char *name)
{
  c_int j, i, row_start, row_stop;
  c_int k = 0;

  // Print name
  c_print("%s :\n", name);

  for (j = 0; j < M->n; j++) {
    row_start = M->p[j];
    row_stop  = M->p[j + 1];

    if (row_start == row_stop) continue;
    else {
      for (i = row_start; i < row_stop; i++) {
        c_print("\t[%3u,%3u] = %.3g\n", (int)M->i[i], (int)j, M->x[k++]);
      }
    }
  }
}

void BarrierCert::calc_qp() {
    // Problem settings
    OSQPSettings* settings =
        reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

    // Structures
    OSQPWorkspace* work;  // Workspace
    OSQPData* osqp_data;  // OSQPData

    // Populate data
    osqp_data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    osqp_data->n = qp_.n;
    osqp_data->m = qp_.m;
    osqp_data->P = csc_matrix(
        osqp_data->n, osqp_data->n, qp_.P_nnz, qp_.P_x.data(),
        qp_.P_i.data(), qp_.P_p.data());
    osqp_data->q = qp_.q.data();
    osqp_data->A = csc_matrix(
        osqp_data->m, osqp_data->n, qp_.A_nnz, qp_.A_x.data(),
        qp_.A_i.data(), qp_.A_p.data());
    osqp_data->l = qp_.l.data();
    osqp_data->u = qp_.u.data();

    // Define Solver settings as default
    osqp_set_default_settings(settings);
    settings->alpha = 1.0; // Change alpha parameter
    settings->verbose = false;
    settings->eps_abs = 1.0e-6;
    settings->eps_rel = 1.0e-6;
    settings->polish = true;
    // settings->max_iter = 1000000;

    // Setup workspace
    work = osqp_setup(osqp_data, settings);

    if (qp_.x_.size() != qp_.n || qp_.y_.size() != qp_.m) {
        qp_.x_.resize(qp_.n);
        qp_.y_.resize(qp_.m);
    } else {
        osqp_warm_start(work, qp_.x_.data(), qp_.y_.data());
    }

    // Solve Problem
    osqp_solve(work);

    std::copy(work->x, work->x + qp_.n, qp_.x_.begin());
    std::copy(work->y, work->y + qp_.m, qp_.y_.begin());
    // print_qp();

    // csc foo;
    // foo.m = qp_.m;
    // foo.n = qp_.n;
    // foo.p = qp_.A_p.data();
    // foo.i = qp_.A_i.data();
    // foo.x = qp_.A_x.data();
    // print_csc_matrix_foo(&foo, "foo");
    //
    // csc foo2;
    // foo2.m = qp_.m;
    // foo2.n = qp_.n;
    // foo2.p = qp_.P_p.data();
    // foo2.i = qp_.P_i.data();
    // foo2.x = qp_.P_x.data();
    // print_csc_matrix_foo(&foo2, "foo2");


    u_safe_.resize(qp_.n);
    std::copy(work->solution->x, work->solution->x + qp_.n, u_safe_.begin());

    // std::cout << "sln: ";
    // br::copy(u_safe_, std::ostream_iterator<double>(std::cout, ", "));
    // std::cout << std::endl;
    // std::cout << "solution: " << work->info->status << std::endl;
    // std::cout << "cost: " << work->info->obj_val << std::endl;

    // Cleanup
    osqp_cleanup(work);
    c_free(osqp_data->A);
    c_free(osqp_data->P);
    c_free(osqp_data);
    c_free(settings);
}

void BarrierCert::set_qp_constraints(std::map<int, std::map<int, double>> &A) {
    auto len = [&](auto &m) {return m.size();};
    int num_entries =
        accumulate(A | ba::map_values | ba::transformed(len), 0);

    qp_.A_nnz = num_entries;
    qp_.A_x.resize(num_entries);
    qp_.A_i.resize(num_entries);
    qp_.A_p.resize(qp_.n + 1);

    int A_idx = 0;
    for (auto &kv : A) {
        int A_c = kv.first;

        qp_.A_p[A_c] = A_idx;
        for (auto &kv2 : kv.second) {
            int A_r = kv2.first;
            qp_.A_x[A_idx] = kv2.second;
            qp_.A_i[A_idx++] = A_r;
        }
    }
    qp_.A_p[qp_.P_nnz] = num_entries;
}

std::vector<unsigned int> BarrierCert::get_ids() {
    std::vector<unsigned int> ids;
    if (centralized_) {
        ids.reserve(u_hat_.size());
        br::copy(u_hat_ | ba::map_keys, std::back_inserter(ids));
    } else {
        ids.push_back(id_);
    }
    return ids;
}

void BarrierCert::set_qp_cost(
        size_t size_u_hat) {
    // general problem: 1/2 x^T P x + q^T x
    // specific problem: 1/2 ||u - u_hat||^2 = 1/2 u^T I u - u_hat^T u

    qp_.n = size_u_hat;

    qp_.P_nnz = qp_.n;
    qp_.P_x.resize(qp_.n);
    qp_.P_i.resize(qp_.n);
    qp_.P_p.resize(qp_.n + 1);

    br::iota(qp_.P_i, 0);
    br::iota(qp_.P_p, 0);
    br::fill(qp_.P_x, 1);

    qp_.q.clear();
    qp_.q.reserve(qp_.n);
    for (auto &vec : u_hat_ | ba::map_values) {
        for (double val : vec) {
            qp_.q.push_back(-val);
        }
    }
}

void BarrierCert::set_actuator_constraints(
        const std::vector<unsigned int> &ids,
        size_t start_row,
        std::map<int, std::map<int, double>> &A) {

    int A_r = start_row;
    int A_c = 0;
    const double inf = std::numeric_limits<double>::infinity();

    for (size_t j = 0; j < ids.size(); j++) {
        auto &lims = fw_params_.lims;
        for (size_t i = 0; i < lims.size(); i++) {
            qp_.l[A_r] = lims[i].first;
            qp_.u[A_r] = inf;
            A[A_c][A_r++] = 1;

            qp_.l[A_r] = -inf;
            qp_.u[A_r] = lims[i].second;
            A[A_c++][A_r++] = 1;
        }
    }

}

void BarrierCert::set_bc_constraints(
        const std::vector<unsigned int> &ids,
        unsigned int start_row,
        std::map<int, std::map<int, double>> &A) {

    int A_r = start_row;

    for (const int id1 : ids) {
        for (auto &kv : cached_data_[id1]) {
            const int id2 = kv.first;
            if (centralized_ && id2 < id1) continue;

            const Data &data = kv.second;

            auto assign = [&](auto &vec, int id){
                int A_c = u_map_.at(id);
                for (const double Lgh : vec) {
                    A[A_c++][A_r] = Lgh;
                }
            };

            assign(data.Lgh_a, id1);

            const double alpha_h = alpha(data.h);
            double b = data.Lfh + alpha_h;

            if (centralized_) {
                assign(data.Lgh_b, id2);
            } else {
                const int card_zeta = 2;
                const double coef = (card_zeta - 1) / static_cast<double>(card_zeta);

                // assuming u_z1, u_z2 are zero here
                const double Lgh_gamma_a =
                    data.Lgh_a[0] * data.gamma1[0] + data.Lgh_a[1] * data.gamma1[1];
                const double Lgh_gamma_b =
                    data.Lgh_b[0] * data.gamma2[0] + data.Lgh_b[1] * data.gamma2[1];
                const double Lgh_gamma = Lgh_gamma_a + Lgh_gamma_b;
                b += Lgh_gamma_b - coef * (data.Lfh + Lgh_gamma + alpha_h);
            }

            qp_.l[A_r] = -b;
            qp_.u[A_r] = std::numeric_limits<double>::infinity();
            A_r++;
        }
    }
}

std::vector<size_t> BarrierCert::u_hat_sizes() {
    std::vector<size_t> sz;
    sz.reserve(u_hat_.size());
    auto get_size = [&](const auto &v) {return v.size();};
    br::transform(u_hat_ | ba::map_values, std::back_inserter(sz), get_size);
    return sz;
}

void BarrierCert::set_u_map(const std::vector<size_t> &sizes) {
    std::vector<int> u_idxs {0};
    boost::partial_sum(sizes, std::back_inserter(u_idxs));
    for (const auto &el : u_hat_ | ba::map_keys | ba::indexed(0)) {
        u_map_[el.value()] = u_idxs[el.index()];
    }
}

std::vector<double> BarrierCert::get_ctrl(int id) {
    auto beg_it = u_safe_.begin() + u_map_.at(id);
    std::vector<double> u_out(beg_it, beg_it + u_hat_.at(id).size());
    return u_out;
}

std::vector<int> BarrierCert::get_neigh(int id1) {
    std::vector<sc::ID> neigh;
    Eigen::Vector3d &pos = contacts_->at(id1).state()->pos();
    rtree_->neighbors_in_range(pos, neigh, sensing_range_, id1);

    if (centralized_) {
        auto lt_id = [&](auto id2) {return id2.id() < id1;};
        neigh.erase(br::remove_if(neigh, lt_id), neigh.end());
    }

    std::vector<int> neigh_int;
    neigh_int.reserve(neigh.size());
    auto to_id = [&](const sc::ID &neigh_id) {return neigh_id.id();};
    br::transform(neigh, std::back_inserter(neigh_int), to_id);

    return neigh_int;
}

void BarrierCert::bc_fw_fw(int id1, int id2, BarrierCert::Data &data) {
    double v1, w1, dz1, v2, w2, dz2;
    std::tie(v1, w1, dz1, v2, w2, dz2) =
        parse_fw_gamma(calc_type_, id1, id2, vel_min_, contacts_, fw_params_.gamma);

    const double nan = std::numeric_limits<double>::infinity();
    double v1_straight_gamma = nan, v2_straight_gamma = nan;
    if (calc_type_ == CalcType::STRAIGHT) {
        v1_straight_gamma = adjust_gamma_straight_vel(id1, straight_pct_offset_, v1, vel_min_, vel_max_);
        v2_straight_gamma = adjust_gamma_straight_vel(id2, straight_pct_offset_, v1, vel_min_, vel_max_);
    }
    const double sigma = v1 / v2;

    auto calc_h = [&](const std::vector<double> &x) {

        double d_sq = calc_type_ == CalcType::STRAIGHT ?
            dist_sq_fw_fw_straight(x, v1_straight_gamma, v2_straight_gamma) :
            dist_sq_fw_fw_turn(x, v2, w1, sigma, delta_, id1 < id2);
        if (d_sq < 0) {
            if (d_sq > -1e-9) {
                // numerical issue fixes for numbers extremely close to 0
                // note: this probably means it is not in the safe set
                // anyway, which should be printed elsewhere
                d_sq = 0;
            } else {
                throw std::runtime_error("cannot take square root of 0 in calc_h");
            }
        }
        // std::cout << "dist is " << sqrt(d_sq) << std::endl;
        double h = sqrt(d_sq) - Ds_;

        const double out = is_gamma_turn(id1, id2) ?
            limited_sensing_.tilde_h(h) : h;
        return out;
    };

    std::vector<double> x;

    for (int _id : {id1, id2}) {
        sc::StatePtr s = contacts_->at(_id).state();
        x.push_back(s->pos()(0));
        x.push_back(s->pos()(1));
        x.push_back(s->quat().yaw());
        if (is_3d_) {
            x.push_back(s->pos()(2));
        }
    }

    data.h = calc_h(x);
    data.Lfh = 0;

    auto ph_px = [&](int idx) {return calc_ph_px(data.h, x, idx, calc_h);};

    const double th1 = x[2];
    const double th2 = is_3d_ ? x[6] : x[5];
    const double ph_px1 = ph_px(0);
    const double ph_py1 = ph_px(1);
    const double ph_pth1 = ph_px(2);
    const double ph_px2 = ph_px(is_3d_ ? 4 : 3);
    const double ph_py2 = ph_px(is_3d_ ? 5 : 4);
    const double ph_pth2 = ph_px(is_3d_ ? 6 : 5);

    data.Lgh_a  = {ph_px1 * cos(th1) + ph_py1 * sin(th1), ph_pth1};
    data.Lgh_b  = {ph_px2 * cos(th2) + ph_py2 * sin(th2), ph_pth2};
    if (is_3d_) {
        const double ph_pz1 = ph_px(3);
        const double ph_pz2 = ph_px(7);
        data.Lgh_a.push_back(ph_pz1);
        data.Lgh_b.push_back(ph_pz2);
    }
}

bool BarrierCert::is_gamma_turn(int id1, int id2) {
    const double eps = 1e-6;
    return std::abs(fw_params_.gamma[1]) > eps;
}

void BarrierCert::init(
        std::map<std::string, std::string> &params,
        int id,
        scrimmage::ContactMapPtr contacts,
        scrimmage::RTreePtr rtree,
        std::shared_ptr<const scrimmage::Time> time) {

    time_ = time;
    for (auto &kv : *contacts) {
        prev_alt_[kv.first] = kv.second.state()->pos()(2);
    }
    delta_ = sc::Angles::deg2rad(std::stod(params.at("delta")));
    draw_paths_ = sc::str2bool(params.at("draw_paths"));
    straight_pct_offset_ = std::stod(params.at("straight_pct_offset"));
    vel_max_ = std::stod(params.at("vel_max"));
    vel_min_ = std::stod(params.at("vel_min"));

    Ds_ = std::stod(params.at("Ds"));
    is_3d_ = sc::str2bool(params.at("is_3d"));
    time_delay_eta_ = std::stod(params.at("time_delay_eta"));

    id_ = id;
    contacts_ = contacts;
    rtree_ = rtree;

    std::tie(calc_type_, fw_params_) = parse_fw_params(params);

    sensing_range_ = std::stod(params.at("sensing_range"));
    if (sensing_range_ < 0) {
        sensing_range_ = std::numeric_limits<double>::infinity();
    }

    centralized_ = sc::str2bool(params.at("centralized"));
    alpha_coef_ = std::stod(params.at("alpha_coef"));

    limited_sensing_ = LimitedSensing(params);
}

void BarrierCert::update_cached_data(const std::vector<unsigned int> &ids) {

    for (int id1 : ids) {
        for (int id2 : get_neigh(id1)) {

            double v1, v2, w;
            std::tie(v1, w, std::ignore, v2, std::ignore, std::ignore) =
                parse_fw_gamma(calc_type_, id1, id2, vel_min_, contacts_, fw_params_.gamma);

            Data data;
            data.id1 = id1;
            data.id2 = id2;

            bc_fw_fw(id1, id2, data);
            // std::cout << "Lgh[" << data.id1 << "] = " << data.Lgh_a[0] << ", " << data.Lgh_a[1] << ", " << ", " << data.Lgh_a[2] << std::endl;
            // std::cout << "Lgh[" << data.id2 << "] = " << data.Lgh_b[0] << ", " << data.Lgh_b[1] << ", " << ", " << data.Lgh_b[2] << std::endl;
            // std::cout << "h[" << data.id1 << ", " << data.id2 << "] = " << data.h << std::endl;
            if (calc_type_ == CalcType::STRAIGHT) {
                const double v1_straight_gamma = adjust_gamma_straight_vel(id1, straight_pct_offset_, v1, vel_min_, vel_max_);
                const double v2_straight_gamma = adjust_gamma_straight_vel(id2, straight_pct_offset_, v1, vel_min_, vel_max_);
                data.gamma1 = {v1_straight_gamma, 0};
                data.gamma2 = {v2_straight_gamma, 0};
            } else {
                data.gamma1 = {v1, w};
                data.gamma2 = {v2, w};
            }
            if (is_3d_) {
                data.gamma1.push_back(0);
                data.gamma2.push_back(0);
            }

            cached_data_[id1][id2] = data;
            cached_data_[id2][id1] = data;
        }
    }
}

void BarrierCert::calc_paths(const std::vector<unsigned int> &ids) {
    if (!draw_paths_) return;

    paths_.clear();
    for (int id1 : ids) {
        using Eigen::Vector2d;
        const Vector2d pos = contacts_->at(id1).state()->pos().head<2>();
        const double yaw = contacts_->at(id1).state()->quat().yaw();
        const Vector2d orient = Vector2d(cos(yaw), sin(yaw));
        const double vel = fw_params_.gamma[0];
        const double turn_rate = fw_params_.gamma[1];
        const size_t num_pts = 1000;

        auto create_path = [&](auto cb) {
            std::vector<scrimmage_proto::Vector3d> path;
            for (size_t i = 0; i < num_pts; i++) {
                Vector2d pt = cb(i);
                scrimmage_proto::Vector3d pt_proto;
                sc::set(&pt_proto, pt(0), pt(1), 0);
                path.push_back(std::move(pt_proto));
            }
            paths_.push_back(std::move(path));
        };

        if (std::abs(turn_rate) < 1e-6) {
            const double time_offset = 500;
            const double dt = time_offset / num_pts;
            create_path([&](size_t i){return Vector2d(pos + i * dt * orient);});
        } else {
            const double radius = std::abs(vel / turn_rate);
            Vector2d offset =
                radius * boost::math::sign(-turn_rate) * (Eigen::Rotation2Dd(M_PI / 2) * orient);
            Vector2d ctr = pos - offset;
            const double d_theta = 2 * M_PI / num_pts;
            create_path([&](size_t i){return Vector2d(ctr + Eigen::Rotation2Dd(i * d_theta) * offset);});
        }
    }
}

void BarrierCert::set_u_hat(int id, const std::vector<double> &u_hat) {
    u_hat_[id] = u_hat;
}

std::map<int, std::map<int, BarrierCert::Data>> &BarrierCert::cached_data() {
    return cached_data_;
}

size_t BarrierCert::get_num_bc(const std::vector<unsigned int> &ids) {
    auto get_size = [&](const auto &v) {return v.size();};
    auto get_cached = [&](int id1) {return cached_data_[id1];};
    int num_bc = accumulate(ids |
        ba::transformed(get_cached) |
        ba::transformed(get_size), 0);
    if (centralized_) num_bc /= 2;
    return num_bc;
}

double BarrierCert::alpha(double h_x) {
    return alpha_coef_ * pow(h_x, 3);
}

}  // namespace barrier_certs
