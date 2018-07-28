#include <barrier-certs/plugins/metrics/BarrierCertMetrics/BarrierCertMetrics.h>
#include <barrier-certs/Utilities.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>

#include <scrimmage/common/CSV.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>

#include <iostream>
#include <iomanip>

#include <boost/range/combine.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/min_element.hpp>

namespace sc = scrimmage;
namespace ba = boost::adaptors;
namespace br = boost::range;
namespace py = pybind11;

REGISTER_PLUGIN(scrimmage::Metrics,
                barrier_certs::metrics::BarrierCertMetrics,
                BarrierCertMetrics_plugin)

namespace barrier_certs {
namespace metrics {

void BarrierCertMetrics::init(std::map<std::string, std::string> &params) {
    CalcType calc_type;
    std::tie(calc_type, fw_params_) = parse_fw_params(params);
    Ds_ = std::stod(params.at("Ds"));

    auto controls_cb = [&](auto &msg) {
        auto &d = msg->data;
        for (auto el : boost::combine(d.u_hat, d.u_safe) | ba::indexed(0)) {
            size_t idx = el.index();
            double u_hat, u_safe;
            boost::tie(u_hat, u_safe) = el.value();

            py::dict dict;
            dict["t"] = d.t;
            dict["veh_id"] = d.veh_id;
            dict["idx"] = idx;
            dict["u_hat"] = u_hat;
            dict["u_safe"] = u_safe;
            data_controls_.append(dict);
        }
    };

    auto safety_cb = [&](auto &msg) {
        auto &d = msg->data;
        py::dict dict;
        dict["t"] = d.t;
        dict["veh_id"] = d.veh_id;
        dict["min_h"] = d.min_h;
        dict["min_h_id"] = d.min_h_id;
        dict["min_d"] = d.min_d;
        dict["min_d_id"] = d.min_d_id;
        data_safety_.append(dict);

        auto it = minimums_.find(d.veh_id);
        if (it == minimums_.end()) {
            minimums_[d.veh_id] = {d.min_h, d.min_d};
        } else {
            double min_h, min_d;
            std::tie(min_h, min_d) = it->second;
            it->second.first = std::min(min_h, d.min_h);
            it->second.second = std::min(min_d, d.min_d);
        }
    };

    subscribe<ControlsData>("GlobalNetwork", "BarrierCertControls", controls_cb);
    subscribe<SafetyData>("GlobalNetwork", "BarrierCertSafety", safety_cb);
}

void BarrierCertMetrics::print_team_summaries() {
    const std::string log_dir = parent_->mp()->log_dir();
    auto write_csv = [&](const std::string &fname, const std::string &headers, auto callback) {
        const std::string fname_ds = log_dir + "/" + fname;
        sc::CSV csv;
        if (!csv.open_output(fname_ds)) {
            std::cout << "could not open " << fname_ds << std::endl;
        } else {
            csv.set_column_headers(headers);
            callback(csv);
            csv.close_output();
        }
    };

    write_csv("Ds.csv", "Ds", [&](auto &csv){csv.append({{"Ds", Ds_}});});

    py::object df = py::module::import("pandas").attr("DataFrame");
    py::object df_controls = df(data_controls_);
    py::object df_safety = df(data_safety_);

    df_controls.attr("to_pickle")(log_dir + "/data_controls.pickle");
    df_safety.attr("to_pickle")(log_dir + "/data_safety.pickle");

    // printout
    if (minimums_.empty()) return;

    auto print_id = [&](auto s){std::cout << std::setw(3) << std::left << s << ": ";};
    auto print_val = [&](auto s){std::cout << std::right << std::fixed << std::setprecision(2) << std::setw(8) << s;};
    auto cmp_h = [&](auto &kv1, auto &kv2) {return kv1.second.first < kv2.second.first;};
    auto cmp_d = [&](auto &kv1, auto &kv2) {return kv1.second.second < kv2.second.second;};

    size_t min_h_id = br::min_element(minimums_, cmp_h)->first;
    size_t min_d_id = br::min_element(minimums_, cmp_d)->first;

    print_id("id");
    print_val("min_h*");
    print_val("min_d");
    std::cout << std::endl;

    for (auto &kv : minimums_) {
        size_t id = kv.first;
        double min_h, min_d;
        std::tie(min_h, min_d) = kv.second;

        print_id(id);
        print_val(min_h);
        print_val(min_d);

        if (id == min_h_id) std::cout << " (min_h)";
        if (id == min_d_id) std::cout << " (min_d)";
        std::cout << std::endl;
    }
    std::cout << "*min_h may be greater than min_d because it is the\n";
    std::cout << "min squared distance in excess of the squared safety distance." << std::endl;
}
} // namespace metrics
} // namespace barrier_certs
