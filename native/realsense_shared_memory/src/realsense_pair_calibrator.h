#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

#include <atomic>
#include <array>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class RealSensePairCalibrator : public godot::RefCounted {
    GDCLASS(RealSensePairCalibrator, godot::RefCounted)

public:
    struct Options {
        std::string mode = "markerless";
        std::string reference_serial;
        std::string target_serial;
        std::string profile = "highres30";
        std::string model_path;
        std::string onnx_backend = "onnx_cuda";
        int marker_id = 49;
        double marker_size_m = 0.15;
        int capture_frames = 10;
        int marker_frames = 24;
        double min_depth_m = 0.2;
        double max_depth_m = 3.0;
        std::array<double, 12> initial_transform = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        };
        bool has_initial_transform = false;
        std::array<double, 12> benchmark_transform = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        };
        bool has_benchmark_transform = false;
        std::array<double, 12> prior_transform = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        };
        bool has_prior_transform = false;
    };

    RealSensePairCalibrator();
    ~RealSensePairCalibrator();

    bool start(const godot::Dictionary &p_options);
    void cancel();
    bool is_running() const;
    godot::String get_status() const;
    godot::Dictionary get_result() const;

protected:
    static void _bind_methods();

private:
    struct Result {
        bool ready = false;
        bool ok = false;
        std::string method;
        std::string status;
        double rotation[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        double translation[3] = {0.0, 0.0, 0.0};
        int matches = 0;
        int inliers = 0;
        double match_median_m = 0.0;
        std::string markerless_solver;
        double markerless_reprojection_px = 0.0;
        double icp_fitness = 0.0;
        double icp_rmse_m = 0.0;
        double refine_translation_m = 0.0;
        double refine_rotation_deg = 0.0;
        double marker_reprojection_px = 0.0;
        int marker_reference_frames = 0;
        int marker_target_frames = 0;
        bool has_marker_reference_transform = false;
        double marker_reference_rotation[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        double marker_reference_translation[3] = {0.0, 0.0, 0.0};
        double initial_rotation[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        double initial_translation[3] = {0.0, 0.0, 0.0};
        double initial_overlap_score = 0.0;
        double refined_overlap_score = 0.0;
        double global_search_score = 0.0;
        double global_overlap_score = 0.0;
        double benchmark_overlap_score = 0.0;
        double benchmark_translation_error_m = 0.0;
        double benchmark_rotation_error_deg = 0.0;
        double prior_overlap_score = 0.0;
        bool refinement_selected = false;
        bool global_search_selected = false;
        bool prior_selected = false;
    };

    mutable std::mutex state_mutex;
    std::thread worker;
    std::atomic<bool> running{false};
    std::atomic<bool> cancel_requested{false};
    std::string status = "Native RealSense calibration is idle";
    Result result;

    void set_status(const std::string &p_status);
    void run(Options p_options);
    void finish(const Result &p_result);
};
