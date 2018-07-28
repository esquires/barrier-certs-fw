#ifndef INCLUDE_BARRIER_CERTS_BARRIERCERT_H_
#define INCLUDE_BARRIER_CERTS_BARRIERCERT_H_

#include <barrier-certs/Utilities.h>
#include <barrier-certs/LimitedSensing.h>

#include <scrimmage/proto/Vector3d.pb.h>

#include <osqp/osqp.h>

#include <map>
#include <unordered_map>
#include <vector>
#include <utility>
#include <memory>
#include <set>
#include <cmath>
#include <string>

namespace scrimmage {
class Contact;
using ContactMap = std::unordered_map<int, Contact>;
using ContactMapPtr = std::shared_ptr<ContactMap>;

class RTree;
using RTreePtr = std::shared_ptr<RTree>;

class Time;
}

namespace barrier_certs {

class BarrierCert {
 public:

    struct Data {
        std::vector<double> Lgh_a;
        std::vector<double> Lgh_b;
        double Lfh;
        double h;
        int id1;
        int id2;
        std::vector<double> gamma1;
        std::vector<double> gamma2;
    };

    void safe_ctrl_input();
    void init(
        std::map<std::string, std::string> &params,
        int id,
        scrimmage::ContactMapPtr contacts,
        scrimmage::RTreePtr rtree,
        std::shared_ptr<const scrimmage::Time> time);

    std::vector<double> get_ctrl(int id);
    void set_u_hat(int id, const std::vector<double> &u);
    std::map<int, std::map<int, Data>> &cached_data();

    std::vector<std::vector<scrimmage_proto::Vector3d>> &paths() {return paths_;}

    LimitedSensing &limited_sensing() {return limited_sensing_;}

 protected:
    struct BarrierCertQP {
        // basic problem size
        size_t n;
        size_t m;

        // vectors for hot start
        std::vector<double> x_;
        std::vector<double> y_;

        // cost matrices
        std::vector<c_float> P_x;   // non-zero values in the matrix
        c_int P_nnz;                // number of non-zero entries
        std::vector<c_int> P_i;     // row index associated with each entry in the matrix
        std::vector<c_int> P_p;     // entry index of P_x where a new column starts
        std::vector<c_float> q;     // linear cost

        // constraint limits
        std::vector<c_float> l;     // lower constraint
        std::vector<c_float> u;     // upper constraint

        // constraint matrices
        std::vector<c_float> A_x;
        c_int A_nnz;
        std::vector<c_int> A_i;
        std::vector<c_int> A_p;
    };

    void update_cached_data(const std::vector<unsigned int> &ids);
    void calc_paths(const std::vector<unsigned int> &ids);
    std::vector<int> get_neigh(int id1);
    std::vector<size_t> u_hat_sizes();
    void set_u_map(const std::vector<size_t> &sizes);
    std::vector<unsigned int> get_ids();
    void set_qp_cost(size_t size_u_hat);
    void set_actuator_constraints(
        const std::vector<unsigned int> &ids,
        size_t start_row,
        std::map<int, std::map<int, double>> &A);
    void set_bc_constraints(
        const std::vector<unsigned int> &ids,
        unsigned int start_row,
        std::map<int, std::map<int, double>> &A);
    void set_qp_constraints(std::map<int, std::map<int, double>> &A);
    void calc_qp();
    void print_qp();
    size_t get_num_bc(const std::vector<unsigned int> &ids);
    std::vector<double> get_mu_vec(
        const std::vector<unsigned int> &ids, size_t num_bc);

    bool is_gamma_turn(int id1, int id2);

    void bc_fw_fw(int id1, int id2, Data &data);
    double alpha(double h_x);

    double delta_ = NAN;

    double sensing_range_;

    CalcType calc_type_;
    scrimmage::RTreePtr rtree_;
    scrimmage::ContactMapPtr contacts_;

    std::unordered_map<int, int> u_map_; // id: starting location in optimization vector
    std::vector<double> u_safe_;
    std::map<int, std::vector<double>> u_hat_;
    std::map<int, std::map<int, Data>> cached_data_; // id1: id2: Data

    double straight_pct_offset_;
    double vel_max_;
    double vel_min_;

    int id_;
    bool centralized_;
    double Ds_;
    double alpha_coef_ = 1;
    VehicleParams fw_params_;

    BarrierCertQP qp_;
    LimitedSensing limited_sensing_;

    bool draw_paths_ = false;
    std::vector<std::vector<scrimmage_proto::Vector3d>> paths_;

    bool is_3d_ = false;
    double time_delay_eta_ = 1;

    std::unordered_map<size_t, double> prev_alt_;
    std::shared_ptr<const scrimmage::Time> time_;
};

}  // namespace barrier_certs

#endif // INCLUDE_BARRIER_CERTS_BARRIERCERT_H_
