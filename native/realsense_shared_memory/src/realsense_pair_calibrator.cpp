#include "realsense_pair_calibrator.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <unordered_map>

#if defined(REALSENSE_DIRECT_ENABLED) && defined(REALSENSE_NATIVE_CALIBRATION_ENABLED)
#include <librealsense2/rs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif

#if defined(REALSENSE_FOUNDATION_STEREO_ENABLED)
#include <onnxruntime/core/session/onnxruntime_c_api.h>
#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#endif

using namespace godot;

namespace {

std::string utf8(const String &p_value) {
    CharString text = p_value.utf8();
    return std::string(text.get_data());
}

#if defined(REALSENSE_DIRECT_ENABLED) && defined(REALSENSE_NATIVE_CALIBRATION_ENABLED)

constexpr double PI = 3.14159265358979323846;

struct Capture {
    int depth_width = 0;
    int depth_height = 0;
    int color_width = 0;
    int color_height = 0;
    cv::Mat raw_depth_m;
    cv::Mat aligned_depth_m;
    cv::Mat color_bgr;
    cv::Mat marker_mask;
    rs2_intrinsics depth_intrinsics{};
    rs2_intrinsics color_intrinsics{};
    cv::Matx44d depth_to_color = cv::Matx44d::eye();
    std::vector<cv::Matx44d> marker_to_depth;
    std::vector<double> marker_reprojection;
};

struct CloudSample {
    cv::Vec3d point;
    cv::Vec3d normal;
    bool has_normal = false;
    cv::Vec3d color;
    bool has_color = false;
};

struct Match3D {
    cv::Vec3d target;
    cv::Vec3d reference;
    double score = 1.0;
    cv::Point2d target_pixel;
    cv::Point2d reference_pixel;
};

struct IcpResult {
    cv::Matx44d transform = cv::Matx44d::eye();
    double fitness = 0.0;
    double rmse = 0.0;
    int correspondences = 0;
};

struct AlignmentScore {
    double score = 0.0;
    double coverage_10mm = 0.0;
    double coverage_20mm = 0.0;
    double coverage_30mm = 0.0;
};

cv::Matx44d extrinsics_matrix(const rs2_extrinsics &p_extrinsics) {
    cv::Matx44d out = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out(row, col) = p_extrinsics.rotation[col * 3 + row];
        }
        out(row, 3) = p_extrinsics.translation[row];
    }
    return out;
}

cv::Matx33d rotation_of(const cv::Matx44d &p_transform) {
    cv::Matx33d out;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out(row, col) = p_transform(row, col);
        }
    }
    return out;
}

cv::Vec3d translation_of(const cv::Matx44d &p_transform) {
    return cv::Vec3d(p_transform(0, 3), p_transform(1, 3), p_transform(2, 3));
}

cv::Vec3d transform_point(const cv::Matx44d &p_transform, const cv::Vec3d &p_point) {
    cv::Vec4d homogeneous(p_point[0], p_point[1], p_point[2], 1.0);
    cv::Vec4d result = p_transform * homogeneous;
    return cv::Vec3d(result[0], result[1], result[2]);
}

double rotation_angle_deg(const cv::Matx44d &p_transform) {
    const cv::Matx33d rotation = rotation_of(p_transform);
    const double trace = rotation(0, 0) + rotation(1, 1) + rotation(2, 2);
    const double cosine = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / PI;
}

std::pair<double, double> transform_delta(const cv::Matx44d &p_from, const cv::Matx44d &p_to) {
    cv::Matx44d delta = p_from.inv() * p_to;
    return {cv::norm(translation_of(delta)), rotation_angle_deg(delta)};
}

cv::Matx44d pose_matrix(const cv::Mat &p_rotation_vector, const cv::Mat &p_translation_vector) {
    cv::Mat rotation;
    cv::Rodrigues(p_rotation_vector, rotation);
    cv::Matx44d out = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out(row, col) = rotation.at<double>(row, col);
        }
        out(row, 3) = p_translation_vector.at<double>(row, 0);
    }
    return out;
}

cv::Mat camera_matrix(const rs2_intrinsics &p_intrinsics) {
    return (cv::Mat_<double>(3, 3) <<
        p_intrinsics.fx, 0.0, p_intrinsics.ppx,
        0.0, p_intrinsics.fy, p_intrinsics.ppy,
        0.0, 0.0, 1.0);
}

bool detect_marker_pose(
    const cv::Mat &p_bgr,
    const rs2_intrinsics &p_intrinsics,
    int p_marker_id,
    double p_marker_size_m,
    cv::Matx44d &r_marker_to_color,
    std::vector<cv::Point2f> *r_corners,
    double &r_reprojection
) {
    cv::Mat gray;
    cv::cvtColor(p_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    parameters->minMarkerPerimeterRate = 0.01;
    parameters->maxMarkerPerimeterRate = 6.0;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(gray, dictionary, corners, ids, parameters);
    int selected = -1;
    double selected_area = 0.0;
    for (int index = 0; index < int(ids.size()); ++index) {
        if (ids[index] != p_marker_id) {
            continue;
        }
        const double area = std::abs(cv::contourArea(corners[index]));
        if (area > selected_area) {
            selected = index;
            selected_area = area;
        }
    }
    if (selected < 0) {
        return false;
    }
    const double half = p_marker_size_m * 0.5;
    std::vector<cv::Point3f> object_points = {
        cv::Point3f(float(-half), float(half), 0.0f),
        cv::Point3f(float(half), float(half), 0.0f),
        cv::Point3f(float(half), float(-half), 0.0f),
        cv::Point3f(float(-half), float(-half), 0.0f),
    };
    cv::Mat rvec;
    cv::Mat tvec;
    bool ok = cv::solvePnP(
        object_points,
        corners[selected],
        camera_matrix(p_intrinsics),
        cv::noArray(),
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE_SQUARE
    );
    if (!ok) {
        return false;
    }
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix(p_intrinsics), cv::noArray(), projected);
    r_reprojection = 0.0;
    for (int index = 0; index < 4; ++index) {
        r_reprojection += cv::norm(projected[index] - corners[selected][index]);
    }
    r_reprojection *= 0.25;
    r_marker_to_color = pose_matrix(rvec, tvec);
    if (r_corners) {
        *r_corners = corners[selected];
    }
    return true;
}

cv::Mat median_depth(const std::vector<cv::Mat> &p_frames) {
    if (p_frames.empty()) {
        return cv::Mat();
    }
    cv::Mat output(p_frames[0].rows, p_frames[0].cols, CV_32F, cv::Scalar(0.0f));
    std::vector<float> values;
    values.reserve(p_frames.size());
    const size_t minimum_support = std::max<size_t>(1, p_frames.size() / 2);
    for (int y = 0; y < output.rows; ++y) {
        float *row = output.ptr<float>(y);
        for (int x = 0; x < output.cols; ++x) {
            values.clear();
            for (const cv::Mat &frame : p_frames) {
                const float value = frame.at<float>(y, x);
                if (std::isfinite(value) && value > 0.0f) {
                    values.push_back(value);
                }
            }
            if (values.size() >= minimum_support) {
                const size_t middle = values.size() / 2;
                std::nth_element(values.begin(), values.begin() + middle, values.end());
                row[x] = values[middle];
            }
        }
    }
    return output;
}

cv::Mat depth_frame_meters(const rs2::depth_frame &p_frame) {
    cv::Mat depth(p_frame.get_height(), p_frame.get_width(), CV_32F);
    const uint16_t *source = static_cast<const uint16_t *>(p_frame.get_data());
    const float units = p_frame.get_units();
    for (int y = 0; y < depth.rows; ++y) {
        float *row = depth.ptr<float>(y);
        for (int x = 0; x < depth.cols; ++x) {
            row[x] = float(source[y * depth.cols + x]) * units;
        }
    }
    return depth;
}

cv::Mat color_frame_bgr(const rs2::video_frame &p_frame) {
    cv::Mat wrapped(
        p_frame.get_height(),
        p_frame.get_width(),
        CV_8UC3,
        const_cast<void *>(p_frame.get_data()),
        p_frame.get_stride_in_bytes()
    );
    return wrapped.clone();
}

void write_bgr_ppm(const std::filesystem::path &p_path, const cv::Mat &p_bgr) {
    if (p_bgr.empty() || p_bgr.type() != CV_8UC3) {
        return;
    }
    std::ofstream file(p_path, std::ios::binary);
    if (!file) {
        return;
    }
    file << "P6\n" << p_bgr.cols << " " << p_bgr.rows << "\n255\n";
    for (int y = 0; y < p_bgr.rows; ++y) {
        const cv::Vec3b *row = p_bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < p_bgr.cols; ++x) {
            const char rgb[3] = {
                char(row[x][2]),
                char(row[x][1]),
                char(row[x][0]),
            };
            file.write(rgb, 3);
        }
    }
}

std::array<int, 3> profile_settings(const std::string &p_profile) {
    if (p_profile == "fast60") {
        return {848, 480, 60};
    }
    if (p_profile == "viewer30") {
        return {848, 480, 30};
    }
    return {1280, 720, 30};
}

bool capture_camera_once(
    const RealSensePairCalibrator::Options &p_options,
    const std::string &p_serial,
    bool p_need_marker,
    std::atomic<bool> &p_cancel,
    Capture &r_capture,
    std::string &r_error
) {
    const std::array<int, 3> settings = profile_settings(p_options.profile);
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_device(p_serial);
    config.enable_stream(RS2_STREAM_DEPTH, settings[0], settings[1], RS2_FORMAT_Z16, settings[2]);
    config.enable_stream(RS2_STREAM_COLOR, settings[0], settings[1], RS2_FORMAT_BGR8, settings[2]);
    try {
        rs2::pipeline_profile profile = pipeline.start(config);
        rs2::align align_to_color(RS2_STREAM_COLOR);
        for (int frame = 0; frame < 60; ++frame) {
            if (p_cancel.load()) {
                pipeline.stop();
                r_error = "calibration cancelled";
                return false;
            }
            pipeline.wait_for_frames(5000);
        }

        std::vector<cv::Mat> raw_depth_frames;
        std::vector<cv::Mat> aligned_depth_frames;
        cv::Mat color_sum;
        int color_sum_frames = 0;
        const int wanted_marker_frames = p_need_marker ? std::max(10, p_options.marker_frames) : 0;
        const int max_frames = std::max(p_options.capture_frames + 20, wanted_marker_frames * 4);
        for (int frame_index = 0; frame_index < max_frames; ++frame_index) {
            if (p_cancel.load()) {
                pipeline.stop();
                r_error = "calibration cancelled";
                return false;
            }
            rs2::frameset frames = pipeline.wait_for_frames(5000);
            rs2::depth_frame raw_depth = frames.get_depth_frame();
            rs2::video_frame raw_color = frames.get_color_frame();
            if (!raw_depth || !raw_color) {
                continue;
            }
            rs2::video_stream_profile depth_profile = raw_depth.get_profile().as<rs2::video_stream_profile>();
            rs2::video_stream_profile color_profile = raw_color.get_profile().as<rs2::video_stream_profile>();
            r_capture.depth_intrinsics = depth_profile.get_intrinsics();
            r_capture.color_intrinsics = color_profile.get_intrinsics();
            r_capture.depth_to_color = extrinsics_matrix(depth_profile.get_extrinsics_to(color_profile));
            r_capture.depth_width = raw_depth.get_width();
            r_capture.depth_height = raw_depth.get_height();
            r_capture.color_width = raw_color.get_width();
            r_capture.color_height = raw_color.get_height();
            cv::Mat bgr = color_frame_bgr(raw_color);
            if (r_capture.color_bgr.empty()) {
                r_capture.color_bgr = bgr.clone();
            }
            if (int(raw_depth_frames.size()) < p_options.capture_frames) {
                raw_depth_frames.push_back(depth_frame_meters(raw_depth));
                if (!p_need_marker) {
                    cv::Mat color_float;
                    bgr.convertTo(color_float, CV_32FC3);
                    if (color_sum.empty()) {
                        color_sum = cv::Mat::zeros(color_float.size(), color_float.type());
                    }
                    color_sum += color_float;
                    ++color_sum_frames;
                }
                rs2::frameset aligned = align_to_color.process(frames);
                rs2::depth_frame aligned_depth = aligned.get_depth_frame();
                if (aligned_depth) {
                    aligned_depth_frames.push_back(depth_frame_meters(aligned_depth));
                }
            }
            if (p_need_marker && int(r_capture.marker_to_depth.size()) < wanted_marker_frames) {
                cv::Matx44d marker_to_color;
                double reprojection = 0.0;
                if (detect_marker_pose(
                    bgr,
                    r_capture.color_intrinsics,
                    p_options.marker_id,
                    p_options.marker_size_m,
                    marker_to_color,
                    nullptr,
                    reprojection
                )) {
                    r_capture.marker_to_depth.push_back(r_capture.depth_to_color.inv() * marker_to_color);
                    r_capture.marker_reprojection.push_back(reprojection);
                    if (reprojection < 2.0) {
                        r_capture.color_bgr = bgr.clone();
                    }
                }
            }
            if (
                int(raw_depth_frames.size()) >= p_options.capture_frames &&
                (!p_need_marker || int(r_capture.marker_to_depth.size()) >= wanted_marker_frames)
            ) {
                break;
            }
        }
        pipeline.stop();
        if (!p_need_marker && color_sum_frames > 0) {
            cv::Mat color_mean = color_sum * (1.0 / color_sum_frames);
            color_mean.convertTo(r_capture.color_bgr, CV_8UC3);
        }
        if (raw_depth_frames.empty() || aligned_depth_frames.empty() || r_capture.color_bgr.empty()) {
            r_error = "camera " + p_serial + " did not produce complete RGB-D frames";
            return false;
        }
        r_capture.raw_depth_m = median_depth(raw_depth_frames);
        r_capture.aligned_depth_m = median_depth(aligned_depth_frames);
        if (p_need_marker && int(r_capture.marker_to_depth.size()) < 8) {
            std::ostringstream message;
            message << "camera " << p_serial << " saw ArUco " << p_options.marker_id << " in only "
                    << r_capture.marker_to_depth.size() << " frames";
            r_error = message.str();
            return false;
        }
        return true;
    } catch (const rs2::error &error) {
        try {
            pipeline.stop();
        } catch (...) {
        }
        r_error = "RealSense capture failed for " + p_serial + ": " + error.what();
        return false;
    } catch (const std::exception &error) {
        try {
            pipeline.stop();
        } catch (...) {
        }
        r_error = "capture failed for " + p_serial + ": " + error.what();
        return false;
    }
}

bool capture_camera(
    const RealSensePairCalibrator::Options &p_options,
    const std::string &p_serial,
    bool p_need_marker,
    std::atomic<bool> &p_cancel,
    Capture &r_capture,
    std::string &r_error
) {
    constexpr int MAX_CAPTURE_ATTEMPTS = 3;
    std::string last_error;
    for (int attempt = 1; attempt <= MAX_CAPTURE_ATTEMPTS; ++attempt) {
        Capture candidate;
        if (capture_camera_once(
            p_options,
            p_serial,
            p_need_marker,
            p_cancel,
            candidate,
            last_error
        )) {
            r_capture = std::move(candidate);
            return true;
        }
        if (p_cancel.load()) {
            r_error = last_error;
            return false;
        }
        if (attempt < MAX_CAPTURE_ATTEMPTS) {
            std::this_thread::sleep_for(std::chrono::milliseconds(750));
        }
    }
    std::ostringstream message;
    message << last_error << " after " << MAX_CAPTURE_ATTEMPTS << " capture attempts";
    r_error = message.str();
    return false;
}

cv::Vec4d rotation_quaternion(const cv::Matx33d &p_rotation) {
    const double trace = p_rotation(0, 0) + p_rotation(1, 1) + p_rotation(2, 2);
    cv::Vec4d quaternion;
    if (trace > 0.0) {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        quaternion = cv::Vec4d(
            0.25 * scale,
            (p_rotation(2, 1) - p_rotation(1, 2)) / scale,
            (p_rotation(0, 2) - p_rotation(2, 0)) / scale,
            (p_rotation(1, 0) - p_rotation(0, 1)) / scale
        );
    } else if (p_rotation(0, 0) > p_rotation(1, 1) && p_rotation(0, 0) > p_rotation(2, 2)) {
        const double scale = std::sqrt(1.0 + p_rotation(0, 0) - p_rotation(1, 1) - p_rotation(2, 2)) * 2.0;
        quaternion = cv::Vec4d(
            (p_rotation(2, 1) - p_rotation(1, 2)) / scale,
            0.25 * scale,
            (p_rotation(0, 1) + p_rotation(1, 0)) / scale,
            (p_rotation(0, 2) + p_rotation(2, 0)) / scale
        );
    } else if (p_rotation(1, 1) > p_rotation(2, 2)) {
        const double scale = std::sqrt(1.0 + p_rotation(1, 1) - p_rotation(0, 0) - p_rotation(2, 2)) * 2.0;
        quaternion = cv::Vec4d(
            (p_rotation(0, 2) - p_rotation(2, 0)) / scale,
            (p_rotation(0, 1) + p_rotation(1, 0)) / scale,
            0.25 * scale,
            (p_rotation(1, 2) + p_rotation(2, 1)) / scale
        );
    } else {
        const double scale = std::sqrt(1.0 + p_rotation(2, 2) - p_rotation(0, 0) - p_rotation(1, 1)) * 2.0;
        quaternion = cv::Vec4d(
            (p_rotation(1, 0) - p_rotation(0, 1)) / scale,
            (p_rotation(0, 2) + p_rotation(2, 0)) / scale,
            (p_rotation(1, 2) + p_rotation(2, 1)) / scale,
            0.25 * scale
        );
    }
    return quaternion * (1.0 / std::sqrt(quaternion.dot(quaternion)));
}

cv::Matx33d quaternion_rotation(cv::Vec4d p_quaternion) {
    p_quaternion *= 1.0 / std::sqrt(p_quaternion.dot(p_quaternion));
    const double w = p_quaternion[0];
    const double x = p_quaternion[1];
    const double y = p_quaternion[2];
    const double z = p_quaternion[3];
    return cv::Matx33d(
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
        2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
        2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)
    );
}

cv::Matx44d average_transforms(const std::vector<cv::Matx44d> &p_transforms) {
    cv::Matx44d output = cv::Matx44d::eye();
    if (p_transforms.empty()) {
        return output;
    }
    cv::Vec4d reference = rotation_quaternion(rotation_of(p_transforms.front()));
    cv::Vec4d quaternion_sum(0.0, 0.0, 0.0, 0.0);
    cv::Vec3d translation_sum(0.0, 0.0, 0.0);
    for (const cv::Matx44d &transform : p_transforms) {
        cv::Vec4d quaternion = rotation_quaternion(rotation_of(transform));
        if (quaternion.dot(reference) < 0.0) {
            quaternion *= -1.0;
        }
        quaternion_sum += quaternion;
        translation_sum += translation_of(transform);
    }
    cv::Matx33d rotation = quaternion_rotation(quaternion_sum);
    cv::Vec3d translation = translation_sum * (1.0 / double(p_transforms.size()));
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            output(row, col) = rotation(row, col);
        }
        output(row, 3) = translation[row];
    }
    return output;
}

cv::Vec3d deproject(const rs2_intrinsics &p_intrinsics, double p_x, double p_y, double p_depth) {
    return cv::Vec3d(
        (p_x - p_intrinsics.ppx) * p_depth / p_intrinsics.fx,
        (p_y - p_intrinsics.ppy) * p_depth / p_intrinsics.fy,
        p_depth
    );
}

double local_depth(const cv::Mat &p_depth, double p_x, double p_y, double p_min, double p_max) {
    const int center_x = int(std::lround(p_x));
    const int center_y = int(std::lround(p_y));
    if (center_x < 0 || center_y < 0 || center_x >= p_depth.cols || center_y >= p_depth.rows) {
        return 0.0;
    }
    const float center = p_depth.at<float>(center_y, center_x);
    const bool center_valid = std::isfinite(center) && center >= p_min && center <= p_max;
    std::vector<float> values;
    for (int y = std::max(0, center_y - 2); y <= std::min(p_depth.rows - 1, center_y + 2); ++y) {
        for (int x = std::max(0, center_x - 2); x <= std::min(p_depth.cols - 1, center_x + 2); ++x) {
            const float value = p_depth.at<float>(y, x);
            if (
                std::isfinite(value) && value >= p_min && value <= p_max &&
                (!center_valid || std::abs(value - center) <= 0.025f)
            ) {
                values.push_back(value);
            }
        }
    }
    if (values.size() < (center_valid ? 3u : 7u)) {
        return 0.0;
    }
    if (!center_valid) {
        std::sort(values.begin(), values.end());
        const float lower = values[values.size() / 4];
        const float upper = values[(values.size() * 3) / 4];
        if (upper - lower > 0.025f) {
            return 0.0;
        }
    }
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    return values[middle];
}

cv::Matx44d rigid_fit(const std::vector<Match3D> &p_matches, const std::vector<int> &p_indices) {
    cv::Matx44d output = cv::Matx44d::eye();
    if (p_indices.size() < 3) {
        return output;
    }
    double weight_sum = 0.0;
    cv::Vec3d target_center(0.0, 0.0, 0.0);
    cv::Vec3d reference_center(0.0, 0.0, 0.0);
    for (int index : p_indices) {
        const double weight = std::max(1e-4, p_matches[index].score);
        weight_sum += weight;
        target_center += p_matches[index].target * weight;
        reference_center += p_matches[index].reference * weight;
    }
    target_center *= 1.0 / weight_sum;
    reference_center *= 1.0 / weight_sum;
    cv::Matx33d covariance = cv::Matx33d::zeros();
    for (int index : p_indices) {
        const Match3D &match = p_matches[index];
        const double weight = std::max(1e-4, match.score);
        cv::Vec3d target = match.target - target_center;
        cv::Vec3d reference = match.reference - reference_center;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                covariance(row, col) += weight * target[row] * reference[col];
            }
        }
    }
    cv::Mat singular_values;
    cv::Mat left;
    cv::Mat right_transpose;
    cv::SVD::compute(cv::Mat(covariance), singular_values, left, right_transpose, cv::SVD::FULL_UV);
    cv::Mat rotation_mat = right_transpose.t() * left.t();
    if (cv::determinant(rotation_mat) < 0.0) {
        right_transpose.row(2) *= -1.0;
        rotation_mat = right_transpose.t() * left.t();
    }
    cv::Matx33d rotation;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation(row, col) = rotation_mat.at<double>(row, col);
            output(row, col) = rotation(row, col);
        }
    }
    cv::Vec3d translation = reference_center - rotation * target_center;
    output(0, 3) = translation[0];
    output(1, 3) = translation[1];
    output(2, 3) = translation[2];
    return output;
}

std::vector<int> transform_inliers(
    const std::vector<Match3D> &p_matches,
    const cv::Matx44d &p_transform,
    double p_threshold,
    double *r_median
) {
    std::vector<int> inliers;
    std::vector<double> errors;
    for (int index = 0; index < int(p_matches.size()); ++index) {
        const double error = cv::norm(transform_point(p_transform, p_matches[index].target) - p_matches[index].reference);
        if (error <= p_threshold) {
            inliers.push_back(index);
            errors.push_back(error);
        }
    }
    if (r_median) {
        if (errors.empty()) {
            *r_median = std::numeric_limits<double>::infinity();
        } else {
            const size_t middle = errors.size() / 2;
            std::nth_element(errors.begin(), errors.begin() + middle, errors.end());
            *r_median = errors[middle];
        }
    }
    return inliers;
}

double match_weight_sum(const std::vector<Match3D> &p_matches, const std::vector<int> &p_indices) {
    double total = 0.0;
    for (int index : p_indices) {
        total += std::max(1e-6, p_matches[index].score);
    }
    return total;
}

bool ransac_rigid(
    const std::vector<Match3D> &p_matches,
    cv::Matx44d &r_transform,
    std::vector<int> &r_inliers,
    double &r_median
) {
    if (p_matches.size() < 12) {
        return false;
    }
    std::mt19937 generator(0x51A17u);
    std::uniform_int_distribution<int> distribution(0, int(p_matches.size()) - 1);
    std::vector<int> best;
    double best_weight = -1.0;
    double best_median = std::numeric_limits<double>::infinity();
    cv::Matx44d best_transform = cv::Matx44d::eye();
    const int iterations = std::min(30000, std::max(8000, int(p_matches.size()) * 150));
    for (int iteration = 0; iteration < iterations; ++iteration) {
        std::vector<int> sample;
        while (sample.size() < 3) {
            const int index = distribution(generator);
            if (std::find(sample.begin(), sample.end(), index) == sample.end()) {
                sample.push_back(index);
            }
        }
        cv::Matx44d candidate = rigid_fit(p_matches, sample);
        double median = 0.0;
        std::vector<int> inliers = transform_inliers(p_matches, candidate, 0.04, &median);
        const double inlier_weight = match_weight_sum(p_matches, inliers);
        if (
            inlier_weight > best_weight + 1e-9 ||
            (std::abs(inlier_weight - best_weight) <= 1e-9 && median < best_median)
        ) {
            best = std::move(inliers);
            best_weight = inlier_weight;
            best_median = median;
            best_transform = candidate;
        }
    }
    if (best.size() < 10) {
        return false;
    }
    for (int pass = 0; pass < 6; ++pass) {
        best_transform = rigid_fit(p_matches, best);
        best = transform_inliers(p_matches, best_transform, pass < 2 ? 0.05 : 0.03, &best_median);
        if (best.size() < 10) {
            return false;
        }
    }
    r_transform = best_transform;
    r_inliers = best;
    r_median = best_median;
    return true;
}

void balance_match_weights(
    std::vector<Match3D> &p_matches,
    int p_target_width,
    int p_target_height,
    int p_reference_width,
    int p_reference_height
) {
    constexpr int columns = 8;
    constexpr int rows = 6;
    constexpr int cells = columns * rows;
    std::array<int, cells> target_counts{};
    std::array<int, cells> reference_counts{};
    auto cell_index = [](const cv::Point2d &p_pixel, int p_width, int p_height) {
        const int x = std::clamp(int(p_pixel.x * columns / std::max(1, p_width)), 0, columns - 1);
        const int y = std::clamp(int(p_pixel.y * rows / std::max(1, p_height)), 0, rows - 1);
        return y * columns + x;
    };
    for (const Match3D &match : p_matches) {
        target_counts[cell_index(match.target_pixel, p_target_width, p_target_height)]++;
        reference_counts[cell_index(match.reference_pixel, p_reference_width, p_reference_height)]++;
    }
    for (Match3D &match : p_matches) {
        const int target_count = target_counts[cell_index(match.target_pixel, p_target_width, p_target_height)];
        const int reference_count = reference_counts[cell_index(match.reference_pixel, p_reference_width, p_reference_height)];
        const double density = std::sqrt(double(std::max(1, target_count) * std::max(1, reference_count)));
        match.score = std::max(0.01, match.score) / density;
    }
}

bool solve_rgbd_pnp(
    const std::vector<Match3D> &p_matches,
    bool p_target_points,
    const rs2_intrinsics &p_image_intrinsics,
    cv::Matx44d &r_transform,
    int &r_inliers
) {
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    object_points.reserve(p_matches.size());
    image_points.reserve(p_matches.size());
    for (const Match3D &match : p_matches) {
        const cv::Vec3d &point = p_target_points ? match.target : match.reference;
        const cv::Point2d &pixel = p_target_points ? match.reference_pixel : match.target_pixel;
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2]) || point[2] <= 0.0) {
            continue;
        }
        object_points.emplace_back(float(point[0]), float(point[1]), float(point[2]));
        image_points.emplace_back(float(pixel.x), float(pixel.y));
    }
    if (object_points.size() < 12) {
        return false;
    }
    cv::Mat rotation_vector;
    cv::Mat translation_vector;
    cv::Mat inliers;
    const bool solved = cv::solvePnPRansac(
        object_points,
        image_points,
        camera_matrix(p_image_intrinsics),
        cv::noArray(),
        rotation_vector,
        translation_vector,
        false,
        1000,
        3.0,
        0.999,
        inliers,
        cv::SOLVEPNP_SQPNP
    );
    if (!solved || inliers.rows < 12) {
        return false;
    }
    std::vector<cv::Point3f> inlier_object_points;
    std::vector<cv::Point2f> inlier_image_points;
    inlier_object_points.reserve(inliers.rows);
    inlier_image_points.reserve(inliers.rows);
    for (int row = 0; row < inliers.rows; ++row) {
        const int index = inliers.at<int>(row, 0);
        if (index < 0 || index >= int(object_points.size())) {
            continue;
        }
        inlier_object_points.push_back(object_points[index]);
        inlier_image_points.push_back(image_points[index]);
    }
    if (inlier_object_points.size() >= 6) {
        cv::solvePnPRefineLM(
            inlier_object_points,
            inlier_image_points,
            camera_matrix(p_image_intrinsics),
            cv::noArray(),
            rotation_vector,
            translation_vector
        );
    }
    r_transform = pose_matrix(rotation_vector, translation_vector);
    r_inliers = int(inlier_object_points.size());
    return true;
}

double pixel_error(
    const cv::Matx44d &p_transform,
    const cv::Vec3d &p_point,
    const cv::Point2d &p_expected,
    const rs2_intrinsics &p_intrinsics
) {
    const cv::Vec3d transformed = transform_point(p_transform, p_point);
    if (!std::isfinite(transformed[2]) || transformed[2] <= 1e-5) {
        return std::numeric_limits<double>::infinity();
    }
    const cv::Point2d projected(
        p_intrinsics.fx * transformed[0] / transformed[2] + p_intrinsics.ppx,
        p_intrinsics.fy * transformed[1] / transformed[2] + p_intrinsics.ppy
    );
    return cv::norm(projected - p_expected);
}

double symmetric_reprojection_error(
    const std::vector<Match3D> &p_matches,
    const cv::Matx44d &p_target_to_reference,
    const rs2_intrinsics &p_target_intrinsics,
    const rs2_intrinsics &p_reference_intrinsics
) {
    const cv::Matx44d reference_to_target = p_target_to_reference.inv();
    std::vector<double> errors;
    errors.reserve(p_matches.size());
    for (const Match3D &match : p_matches) {
        const double forward = pixel_error(
            p_target_to_reference,
            match.target,
            match.reference_pixel,
            p_reference_intrinsics
        );
        const double reverse = pixel_error(
            reference_to_target,
            match.reference,
            match.target_pixel,
            p_target_intrinsics
        );
        if (std::isfinite(forward) && std::isfinite(reverse)) {
            errors.push_back(0.5 * (forward + reverse));
        }
    }
    if (errors.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    const size_t middle = errors.size() / 2;
    std::nth_element(errors.begin(), errors.begin() + middle, errors.end());
    return errors[middle];
}

bool solve_rgb_essential(
    const std::vector<Match3D> &p_matches,
    const rs2_intrinsics &p_target_intrinsics,
    const rs2_intrinsics &p_reference_intrinsics,
    cv::Matx44d &r_direction_transform,
    cv::Matx44d &r_median_transform,
    int &r_inliers
) {
    if (p_matches.size() < 20) {
        return false;
    }
    std::vector<cv::Point2f> target_points;
    std::vector<cv::Point2f> reference_points;
    target_points.reserve(p_matches.size());
    reference_points.reserve(p_matches.size());
    for (const Match3D &match : p_matches) {
        target_points.emplace_back(
            float((match.target_pixel.x - p_target_intrinsics.ppx) / p_target_intrinsics.fx),
            float((match.target_pixel.y - p_target_intrinsics.ppy) / p_target_intrinsics.fy)
        );
        reference_points.emplace_back(
            float((match.reference_pixel.x - p_reference_intrinsics.ppx) / p_reference_intrinsics.fx),
            float((match.reference_pixel.y - p_reference_intrinsics.ppy) / p_reference_intrinsics.fy)
        );
    }
    cv::Mat mask;
    cv::Mat essential = cv::findEssentialMat(
        target_points,
        reference_points,
        1.0,
        cv::Point2d(0.0, 0.0),
        cv::RANSAC,
        0.999,
        0.0015,
        mask
    );
    if (essential.empty()) {
        return false;
    }
    cv::Mat rotation;
    cv::Mat translation_direction;
    const int pose_inliers = cv::recoverPose(
        essential,
        target_points,
        reference_points,
        rotation,
        translation_direction,
        1.0,
        cv::Point2d(0.0, 0.0),
        mask
    );
    if (pose_inliers < 20 || rotation.rows != 3 || translation_direction.rows != 3) {
        return false;
    }
    cv::Matx33d rotation_matrix;
    cv::Vec3d direction;
    for (int row = 0; row < 3; ++row) {
        direction[row] = translation_direction.at<double>(row, 0);
        for (int col = 0; col < 3; ++col) {
            rotation_matrix(row, col) = rotation.at<double>(row, col);
        }
    }
    const double direction_norm = cv::norm(direction);
    if (!std::isfinite(direction_norm) || direction_norm < 1e-8) {
        return false;
    }
    direction *= 1.0 / direction_norm;
    std::vector<double> scales;
    std::array<std::vector<double>, 3> translation_components;
    scales.reserve(p_matches.size());
    for (int index = 0; index < int(p_matches.size()); ++index) {
        if (!mask.empty() && mask.at<uint8_t>(index, 0) == 0) {
            continue;
        }
        const cv::Vec3d offset = p_matches[index].reference - rotation_matrix * p_matches[index].target;
        scales.push_back(offset.dot(direction));
        for (int axis = 0; axis < 3; ++axis) {
            translation_components[axis].push_back(offset[axis]);
        }
    }
    if (scales.size() < 20) {
        return false;
    }
    const size_t middle = scales.size() / 2;
    std::nth_element(scales.begin(), scales.begin() + middle, scales.end());
    const double scale = scales[middle];
    cv::Vec3d median_translation;
    for (int axis = 0; axis < 3; ++axis) {
        std::vector<double> &values = translation_components[axis];
        const size_t axis_middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + axis_middle, values.end());
        median_translation[axis] = values[axis_middle];
    }
    r_direction_transform = cv::Matx44d::eye();
    r_median_transform = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            r_direction_transform(row, col) = rotation_matrix(row, col);
            r_median_transform(row, col) = rotation_matrix(row, col);
        }
        r_direction_transform(row, 3) = direction[row] * scale;
        r_median_transform(row, 3) = median_translation[row];
    }
    r_inliers = int(scales.size());
    return true;
}

struct VoxelKey {
    int x;
    int y;
    int z;
    bool operator==(const VoxelKey &p_other) const {
        return x == p_other.x && y == p_other.y && z == p_other.z;
    }
};

struct VoxelHash {
    size_t operator()(const VoxelKey &p_key) const {
        size_t value = size_t(uint32_t(p_key.x) * 73856093u);
        value ^= size_t(uint32_t(p_key.y) * 19349663u);
        value ^= size_t(uint32_t(p_key.z) * 83492791u);
        return value;
    }
};

int nearest_cloud_sample(
    const cv::Vec3d &p_point,
    const std::vector<CloudSample> &p_samples,
    const std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> &p_grid,
    double p_cell_size,
    double p_max_distance_sq,
    bool p_require_normal,
    const cv::Vec3d *p_query_color = nullptr,
    double p_max_color_distance_sq = std::numeric_limits<double>::infinity(),
    double p_color_spatial_weight = 0.0
) {
    const VoxelKey center{
        int(std::floor(p_point[0] / p_cell_size)),
        int(std::floor(p_point[1] / p_cell_size)),
        int(std::floor(p_point[2] / p_cell_size)),
    };
    int best_index = -1;
    double best_score = std::numeric_limits<double>::infinity();
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const auto found = p_grid.find(VoxelKey{center.x + dx, center.y + dy, center.z + dz});
                if (found == p_grid.end()) {
                    continue;
                }
                for (int index : found->second) {
                    if (p_require_normal && !p_samples[index].has_normal) {
                        continue;
                    }
                    const cv::Vec3d difference = p_point - p_samples[index].point;
                    const double distance_sq = difference.dot(difference);
                    if (distance_sq >= p_max_distance_sq) {
                        continue;
                    }
                    double color_distance_sq = 0.0;
                    if (p_query_color != nullptr) {
                        if (!p_samples[index].has_color) {
                            continue;
                        }
                        const cv::Vec3d color_difference = *p_query_color - p_samples[index].color;
                        color_distance_sq = color_difference.dot(color_difference);
                        if (color_distance_sq > p_max_color_distance_sq) {
                            continue;
                        }
                    }
                    const double score = distance_sq + p_color_spatial_weight * color_distance_sq;
                    if (score < best_score) {
                        best_score = score;
                        best_index = index;
                    }
                }
            }
        }
    }
    return best_index;
}

bool depth_point_color(const Capture &p_capture, const cv::Vec3d &p_depth_point, cv::Vec3d &r_color) {
    if (p_capture.color_bgr.empty()) {
        return false;
    }
    const cv::Vec3d color_point = transform_point(p_capture.depth_to_color, p_depth_point);
    if (!std::isfinite(color_point[2]) || color_point[2] <= 1e-5) {
        return false;
    }
    const int x = int(std::lround(
        p_capture.color_intrinsics.fx * color_point[0] / color_point[2] + p_capture.color_intrinsics.ppx
    ));
    const int y = int(std::lround(
        p_capture.color_intrinsics.fy * color_point[1] / color_point[2] + p_capture.color_intrinsics.ppy
    ));
    if (x < 0 || y < 0 || x >= p_capture.color_bgr.cols || y >= p_capture.color_bgr.rows) {
        return false;
    }
    const cv::Vec3b bgr = p_capture.color_bgr.at<cv::Vec3b>(y, x);
    r_color = cv::Vec3d(bgr[0] / 255.0, bgr[1] / 255.0, bgr[2] / 255.0);
    return true;
}

std::vector<CloudSample> organized_cloud(const Capture &p_capture, int p_stride, double p_min, double p_max) {
    std::vector<CloudSample> samples;
    const cv::Mat &depth = p_capture.raw_depth_m;
    samples.reserve(size_t(depth.rows / p_stride) * size_t(depth.cols / p_stride));
    auto point_at = [&](int x, int y, cv::Vec3d &r_point) -> bool {
        if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) {
            return false;
        }
        const double z = depth.at<float>(y, x);
        if (!std::isfinite(z) || z < p_min || z > p_max) {
            return false;
        }
        r_point = deproject(p_capture.depth_intrinsics, x, y, z);
        return true;
    };
    for (int y = p_stride; y + p_stride < depth.rows; y += p_stride) {
        for (int x = p_stride; x + p_stride < depth.cols; x += p_stride) {
            cv::Vec3d point;
            if (!point_at(x, y, point)) {
                continue;
            }
            CloudSample sample;
            sample.point = point;
            cv::Vec3d right;
            cv::Vec3d down;
            if (
                point_at(x + p_stride, y, right) &&
                point_at(x, y + p_stride, down) &&
                std::abs(right[2] - point[2]) < 0.12 &&
                std::abs(down[2] - point[2]) < 0.12
            ) {
                cv::Vec3d normal = (right - point).cross(down - point);
                const double length = cv::norm(normal);
                if (length > 1e-8) {
                    sample.normal = normal * (1.0 / length);
                    sample.has_normal = true;
                }
            }
            samples.push_back(sample);
        }
    }
    return samples;
}

std::vector<CloudSample> voxel_cloud(
    const Capture &p_capture,
    double p_voxel_size,
    double p_min,
    double p_max
) {
    struct VoxelAccum {
        cv::Vec3d point_sum = cv::Vec3d(0.0, 0.0, 0.0);
        cv::Vec3d normal_sum = cv::Vec3d(0.0, 0.0, 0.0);
        cv::Vec3d color_sum = cv::Vec3d(0.0, 0.0, 0.0);
        int point_count = 0;
        int normal_count = 0;
        int color_count = 0;
    };
    const cv::Mat &depth = p_capture.raw_depth_m;
    std::unordered_map<VoxelKey, VoxelAccum, VoxelHash> voxels;
    voxels.reserve(size_t(depth.rows * depth.cols) / 12);
    auto point_at = [&](int x, int y, cv::Vec3d &r_point) -> bool {
        if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) {
            return false;
        }
        const double z = depth.at<float>(y, x);
        if (!std::isfinite(z) || z < p_min || z > p_max) {
            return false;
        }
        r_point = deproject(p_capture.depth_intrinsics, x, y, z);
        return true;
    };
    for (int y = 0; y + 1 < depth.rows; ++y) {
        for (int x = 0; x + 1 < depth.cols; ++x) {
            cv::Vec3d point;
            if (!point_at(x, y, point)) {
                continue;
            }
            const VoxelKey key{
                int(std::floor(point[0] / p_voxel_size)),
                int(std::floor(point[1] / p_voxel_size)),
                int(std::floor(point[2] / p_voxel_size)),
            };
            VoxelAccum &accum = voxels[key];
            accum.point_sum += point;
            accum.point_count++;
            cv::Vec3d color;
            if (depth_point_color(p_capture, point, color)) {
                accum.color_sum += color;
                accum.color_count++;
            }
            cv::Vec3d right;
            cv::Vec3d down;
            if (
                point_at(x + 1, y, right) &&
                point_at(x, y + 1, down) &&
                std::abs(right[2] - point[2]) < 0.08 &&
                std::abs(down[2] - point[2]) < 0.08
            ) {
                cv::Vec3d normal = (right - point).cross(down - point);
                const double length = cv::norm(normal);
                if (length > 1e-8) {
                    accum.normal_sum += normal * (1.0 / length);
                    accum.normal_count++;
                }
            }
        }
    }
    std::vector<CloudSample> output;
    output.reserve(voxels.size());
    for (const auto &entry : voxels) {
        const VoxelAccum &accum = entry.second;
        if (accum.point_count <= 0) {
            continue;
        }
        CloudSample sample;
        sample.point = accum.point_sum * (1.0 / double(accum.point_count));
        if (accum.color_count > 0) {
            sample.color = accum.color_sum * (1.0 / double(accum.color_count));
            sample.has_color = true;
        }
        const double normal_length = cv::norm(accum.normal_sum);
        if (accum.normal_count > 0 && normal_length > 1e-8) {
            sample.normal = accum.normal_sum * (1.0 / normal_length);
            sample.has_normal = true;
        }
        output.push_back(sample);
    }
    const double normal_radius = p_voxel_size * 3.0;
    const double normal_radius_sq = normal_radius * normal_radius;
    std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> normal_grid;
    normal_grid.reserve(output.size());
    for (int index = 0; index < int(output.size()); ++index) {
        const cv::Vec3d &point = output[index].point;
        normal_grid[VoxelKey{
            int(std::floor(point[0] / normal_radius)),
            int(std::floor(point[1] / normal_radius)),
            int(std::floor(point[2] / normal_radius)),
        }].push_back(index);
    }
    for (int sample_index = 0; sample_index < int(output.size()); ++sample_index) {
        const cv::Vec3d &point = output[sample_index].point;
        const VoxelKey center{
            int(std::floor(point[0] / normal_radius)),
            int(std::floor(point[1] / normal_radius)),
            int(std::floor(point[2] / normal_radius)),
        };
        std::vector<std::pair<double, int>> neighbors;
        neighbors.reserve(96);
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const auto found = normal_grid.find(VoxelKey{center.x + dx, center.y + dy, center.z + dz});
                    if (found == normal_grid.end()) {
                        continue;
                    }
                    for (int candidate_index : found->second) {
                        const cv::Vec3d difference = output[candidate_index].point - point;
                        const double distance_sq = difference.dot(difference);
                        if (distance_sq <= normal_radius_sq) {
                            neighbors.emplace_back(distance_sq, candidate_index);
                        }
                    }
                }
            }
        }
        if (neighbors.size() < 6) {
            output[sample_index].has_normal = false;
            continue;
        }
        if (neighbors.size() > 40) {
            std::nth_element(neighbors.begin(), neighbors.begin() + 40, neighbors.end());
            neighbors.resize(40);
        }
        cv::Vec3d center_point(0.0, 0.0, 0.0);
        for (const auto &neighbor : neighbors) {
            center_point += output[neighbor.second].point;
        }
        center_point *= 1.0 / double(neighbors.size());
        cv::Matx33d covariance = cv::Matx33d::zeros();
        for (const auto &neighbor : neighbors) {
            const cv::Vec3d offset = output[neighbor.second].point - center_point;
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    covariance(row, col) += offset[row] * offset[col];
                }
            }
        }
        cv::Mat eigenvalues;
        cv::Mat eigenvectors;
        if (!cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors)) {
            output[sample_index].has_normal = false;
            continue;
        }
        cv::Vec3d normal(
            eigenvectors.at<double>(2, 0),
            eigenvectors.at<double>(2, 1),
            eigenvectors.at<double>(2, 2)
        );
        const double length = cv::norm(normal);
        output[sample_index].normal = normal * (1.0 / std::max(length, 1e-12));
        output[sample_index].has_normal = length > 1e-8;
    }
    return output;
}

void write_cloud_ply(const std::filesystem::path &p_path, const std::vector<CloudSample> &p_cloud) {
    std::ofstream file(p_path);
    if (!file) {
        return;
    }
    file << "ply\nformat ascii 1.0\nelement vertex " << p_cloud.size()
         << "\nproperty double x\nproperty double y\nproperty double z"
         << "\nproperty double nx\nproperty double ny\nproperty double nz\nend_header\n";
    file << std::setprecision(12);
    for (const CloudSample &sample : p_cloud) {
        const cv::Vec3d normal = sample.has_normal ? sample.normal : cv::Vec3d(0.0, 0.0, 0.0);
        file << sample.point[0] << ' ' << sample.point[1] << ' ' << sample.point[2] << ' '
             << normal[0] << ' ' << normal[1] << ' ' << normal[2] << '\n';
    }
}

AlignmentScore alignment_score(
    const Capture &p_reference,
    const Capture &p_target,
    const cv::Matx44d &p_transform,
    double p_min_depth,
    double p_max_depth
) {
    constexpr double voxel_size = 0.012;
    constexpr double search_radius = 0.045;
    std::vector<CloudSample> reference = voxel_cloud(p_reference, voxel_size, p_min_depth, p_max_depth);
    std::vector<CloudSample> target = voxel_cloud(p_target, voxel_size, p_min_depth, p_max_depth);
    for (CloudSample &sample : target) {
        sample.point = transform_point(p_transform, sample.point);
    }
    std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> reference_grid;
    std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> target_grid;
    reference_grid.reserve(reference.size());
    target_grid.reserve(target.size());
    auto add_to_grid = [](const std::vector<CloudSample> &p_cloud, auto &r_grid) {
        for (int index = 0; index < int(p_cloud.size()); ++index) {
            const cv::Vec3d &point = p_cloud[index].point;
            r_grid[VoxelKey{
                int(std::floor(point[0] / search_radius)),
                int(std::floor(point[1] / search_radius)),
                int(std::floor(point[2] / search_radius)),
            }].push_back(index);
        }
    };
    add_to_grid(reference, reference_grid);
    add_to_grid(target, target_grid);
    std::array<double, 3> thresholds = {0.010, 0.020, 0.030};
    std::array<int64_t, 3> hits = {0, 0, 0};
    int64_t total = 0;
    const double max_distance_sq = search_radius * search_radius;
    auto accumulate = [&](const std::vector<CloudSample> &p_queries,
                          const std::vector<CloudSample> &p_candidates,
                          const auto &p_grid) {
        for (const CloudSample &query : p_queries) {
            const int nearest = nearest_cloud_sample(
                query.point, p_candidates, p_grid, search_radius, max_distance_sq, false
            );
            const double distance = nearest >= 0
                ? cv::norm(query.point - p_candidates[nearest].point)
                : std::numeric_limits<double>::infinity();
            for (int index = 0; index < int(thresholds.size()); ++index) {
                if (distance <= thresholds[index]) {
                    hits[index]++;
                }
            }
            total++;
        }
    };
    accumulate(target, reference, reference_grid);
    accumulate(reference, target, target_grid);
    AlignmentScore output;
    if (total <= 0) {
        return output;
    }
    output.coverage_10mm = double(hits[0]) / double(total);
    output.coverage_20mm = double(hits[1]) / double(total);
    output.coverage_30mm = double(hits[2]) / double(total);
    output.score =
        0.20 * output.coverage_10mm +
        0.35 * output.coverage_20mm +
        0.45 * output.coverage_30mm;
    return output;
}

double one_way_alignment_score(
    const std::vector<CloudSample> &p_reference,
    const std::vector<CloudSample> &p_target,
    const std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> &p_reference_grid,
    const cv::Matx44d &p_transform,
    double p_cell_size
) {
    const double max_distance_sq = p_cell_size * p_cell_size;
    std::array<int64_t, 3> hits = {0, 0, 0};
    int64_t total = 0;
    const int sample_step = std::max(1, int(p_target.size() / 30000));
    for (int index = 0; index < int(p_target.size()); index += sample_step) {
        const CloudSample &sample = p_target[index];
        const cv::Vec3d point = transform_point(p_transform, sample.point);
        const cv::Vec3d *query_color = sample.has_color ? &sample.color : nullptr;
        const int nearest = nearest_cloud_sample(
            point,
            p_reference,
            p_reference_grid,
            p_cell_size,
            max_distance_sq,
            false,
            query_color,
            0.36,
            0.000225
        );
        const double distance = nearest >= 0
            ? cv::norm(point - p_reference[nearest].point)
            : std::numeric_limits<double>::infinity();
        if (distance <= 0.010) {
            hits[0]++;
        }
        if (distance <= 0.020) {
            hits[1]++;
        }
        if (distance <= 0.030) {
            hits[2]++;
        }
        total++;
    }
    if (total <= 0) {
        return 0.0;
    }
    return (
        0.25 * double(hits[0]) +
        0.35 * double(hits[1]) +
        0.40 * double(hits[2])
    ) / double(total);
}

cv::Matx44d perturb_alignment(const cv::Matx44d &p_transform, int p_axis, double p_step) {
    cv::Matx44d candidate = p_transform;
    if (p_axis < 3) {
        cv::Vec3d rotation_vector(0.0, 0.0, 0.0);
        rotation_vector[p_axis] = p_step;
        cv::Mat rotation;
        cv::Rodrigues(rotation_vector, rotation);
        cv::Matx33d delta;
        cv::Matx33d current;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                delta(row, col) = rotation.at<double>(row, col);
                current(row, col) = p_transform(row, col);
            }
        }
        const cv::Matx33d updated = delta * current;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                candidate(row, col) = updated(row, col);
            }
        }
    } else {
        candidate(p_axis - 3, 3) += p_step;
    }
    return candidate;
}

cv::Matx44d global_axis_refine(
    const Capture &p_reference,
    const Capture &p_target,
    const cv::Matx44d &p_start,
    double p_min_depth,
    double p_max_depth,
    std::atomic<bool> &p_cancel,
    double &r_search_score
) {
    constexpr double voxel_size = 0.015;
    constexpr double search_radius = 0.045;
    std::vector<CloudSample> reference = voxel_cloud(
        p_reference, voxel_size, p_min_depth, p_max_depth
    );
    std::vector<CloudSample> target = voxel_cloud(
        p_target, voxel_size, p_min_depth, p_max_depth
    );
    std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> reference_grid;
    reference_grid.reserve(reference.size());
    for (int index = 0; index < int(reference.size()); ++index) {
        const cv::Vec3d &point = reference[index].point;
        reference_grid[VoxelKey{
            int(std::floor(point[0] / search_radius)),
            int(std::floor(point[1] / search_radius)),
            int(std::floor(point[2] / search_radius)),
        }].push_back(index);
    }
    cv::Matx44d best = p_start;
    double best_score = one_way_alignment_score(
        reference, target, reference_grid, best, search_radius
    );
    const double beam_rotation_step = 0.9 * PI / 180.0;
    for (int x_step = -1; x_step <= 1; ++x_step) {
        for (int y_step = -1; y_step <= 1; ++y_step) {
            for (int z_step = -1; z_step <= 1; ++z_step) {
                if (x_step == 0 && y_step == 0 && z_step == 0) {
                    continue;
                }
                if (p_cancel.load()) {
                    r_search_score = best_score;
                    return best;
                }
                cv::Matx44d candidate = p_start;
                candidate = perturb_alignment(candidate, 0, x_step * beam_rotation_step);
                candidate = perturb_alignment(candidate, 1, y_step * beam_rotation_step);
                candidate = perturb_alignment(candidate, 2, z_step * beam_rotation_step);
                const auto delta = transform_delta(p_start, candidate);
                if (delta.second > 3.0) {
                    continue;
                }
                const double score = one_way_alignment_score(
                    reference, target, reference_grid, candidate, search_radius
                );
                if (score > best_score + 1e-6) {
                    best = candidate;
                    best_score = score;
                }
            }
        }
    }
    double rotation_step = 0.45 * PI / 180.0;
    double translation_step = 0.0045;
    for (int level = 0; level < 5; ++level) {
        for (int sweep = 0; sweep < 3; ++sweep) {
            bool improved = false;
            for (int axis = 0; axis < 6; ++axis) {
                cv::Matx44d axis_best = best;
                double axis_score = best_score;
                const double step = axis < 3 ? rotation_step : translation_step;
                for (double direction : {-1.0, 1.0}) {
                    if (p_cancel.load()) {
                        r_search_score = best_score;
                        return best;
                    }
                    const cv::Matx44d candidate = perturb_alignment(best, axis, direction * step);
                    const auto delta = transform_delta(p_start, candidate);
                    if (delta.first > 0.025 || delta.second > 3.0) {
                        continue;
                    }
                    const double score = one_way_alignment_score(
                        reference, target, reference_grid, candidate, search_radius
                    );
                    if (score > axis_score + 1e-6) {
                        axis_best = candidate;
                        axis_score = score;
                    }
                }
                if (axis_score > best_score + 1e-6) {
                    best = axis_best;
                    best_score = axis_score;
                    improved = true;
                }
            }
            if (!improved) {
                break;
            }
        }
        rotation_step *= 0.5;
        translation_step *= 0.5;
    }
    r_search_score = best_score;
    return best;
}

cv::Matx44d twist_transform(const cv::Vec<double, 6> &p_delta) {
    cv::Mat rotation;
    cv::Rodrigues(cv::Vec3d(p_delta[0], p_delta[1], p_delta[2]), rotation);
    cv::Matx44d output = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            output(row, col) = rotation.at<double>(row, col);
        }
        output(row, 3) = p_delta[3 + row];
    }
    return output;
}

IcpResult robust_icp(
    const Capture &p_reference,
    const Capture &p_target,
    const cv::Matx44d &p_initial,
    double p_min_depth,
    double p_max_depth,
    std::atomic<bool> &p_cancel
) {
    struct Stage {
        double voxel;
        double correspondence;
        int iterations;
    };
    const std::array<Stage, 3> stages = {{{0.025, 0.055, 60}, {0.01375, 0.03025, 80}, {0.008, 0.0176, 100}}};
    cv::Matx44d transform = p_initial;
    IcpResult output;
    int stage_index = 0;
    for (const Stage &stage : stages) {
        std::vector<CloudSample> reference = voxel_cloud(p_reference, stage.voxel, p_min_depth, p_max_depth);
        std::vector<CloudSample> target = voxel_cloud(p_target, stage.voxel, p_min_depth, p_max_depth);
        const char *debug_dir = std::getenv("REALSENSE_CALIBRATION_DEBUG_DIR");
        if (debug_dir && debug_dir[0]) {
            std::filesystem::create_directories(debug_dir);
            write_cloud_ply(std::filesystem::path(debug_dir) / ("reference_stage_" + std::to_string(stage_index) + ".ply"), reference);
            write_cloud_ply(std::filesystem::path(debug_dir) / ("target_stage_" + std::to_string(stage_index) + ".ply"), target);
        }
        std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> grid;
        grid.reserve(reference.size());
        for (int index = 0; index < int(reference.size()); ++index) {
            const cv::Vec3d &point = reference[index].point;
            VoxelKey key{
                int(std::floor(point[0] / stage.correspondence)),
                int(std::floor(point[1] / stage.correspondence)),
                int(std::floor(point[2] / stage.correspondence)),
            };
            grid[key].push_back(index);
        }
        for (int iteration = 0; iteration < stage.iterations; ++iteration) {
            if (p_cancel.load()) {
                output.transform = transform;
                return output;
            }
            cv::Matx<double, 6, 6> normal_matrix = cv::Matx<double, 6, 6>::zeros();
            cv::Vec<double, 6> right_hand = cv::Vec<double, 6>::all(0.0);
            int correspondence_count = 0;
            double squared_error = 0.0;
            const double max_distance_sq = stage.correspondence * stage.correspondence;
            const double tukey = std::max(0.012, stage.voxel * 1.25);
            constexpr double max_color_distance_sq = 0.25;
            constexpr double color_spatial_weight = 0.0004;
            const int sample_step = std::max(1, int(target.size() / 35000));
            for (int target_index = 0; target_index < int(target.size()); target_index += sample_step) {
                const cv::Vec3d point = transform_point(transform, target[target_index].point);
                const cv::Vec3d *query_color = target[target_index].has_color
                    ? &target[target_index].color
                    : nullptr;
                const int best_index = nearest_cloud_sample(
                    point,
                    reference,
                    grid,
                    stage.correspondence,
                    max_distance_sq,
                    true,
                    query_color,
                    max_color_distance_sq,
                    color_spatial_weight
                );
                if (best_index < 0) {
                    continue;
                }
                const cv::Vec3d normal = reference[best_index].normal;
                const double residual = normal.dot(point - reference[best_index].point);
                const double ratio = std::abs(residual) / tukey;
                if (ratio >= 1.0) {
                    continue;
                }
                const double weight = std::pow(1.0 - ratio * ratio, 2.0);
                const cv::Vec3d rotational = point.cross(normal);
                const cv::Vec<double, 6> jacobian(
                    rotational[0], rotational[1], rotational[2], normal[0], normal[1], normal[2]
                );
                for (int row = 0; row < 6; ++row) {
                    right_hand[row] -= weight * jacobian[row] * residual;
                    for (int col = 0; col < 6; ++col) {
                        normal_matrix(row, col) += weight * jacobian[row] * jacobian[col];
                    }
                }
                correspondence_count++;
                squared_error += residual * residual;
            }
            if (correspondence_count < 60) {
                break;
            }
            cv::Vec<double, 6> delta;
            cv::Mat delta_matrix(6, 1, CV_64F, delta.val);
            for (int diagonal = 0; diagonal < 6; ++diagonal) {
                normal_matrix(diagonal, diagonal) += 1e-7;
            }
            if (!cv::solve(cv::Mat(normal_matrix), cv::Mat(right_hand), delta_matrix, cv::DECOMP_CHOLESKY)) {
                break;
            }
            const double rotation_step = cv::norm(cv::Vec3d(delta[0], delta[1], delta[2]));
            const double translation_step = cv::norm(cv::Vec3d(delta[3], delta[4], delta[5]));
            if (rotation_step > 0.012) {
                for (int index = 0; index < 3; ++index) {
                    delta[index] *= 0.012 / rotation_step;
                }
            }
            if (translation_step > 0.006) {
                for (int index = 3; index < 6; ++index) {
                    delta[index] *= 0.006 / translation_step;
                }
            }
            const cv::Matx44d candidate = twist_transform(delta) * transform;
            const auto total_delta = transform_delta(p_initial, candidate);
            if (total_delta.first > 0.025 || total_delta.second > 3.0) {
                break;
            }
            transform = candidate;
            output.correspondences = correspondence_count;
            output.fitness = double(correspondence_count) / double(std::max<size_t>(1, target.size() / sample_step));
            output.rmse = std::sqrt(squared_error / double(correspondence_count));
            const double magnitude = cv::norm(cv::Vec3d(delta[0], delta[1], delta[2])) + cv::norm(cv::Vec3d(delta[3], delta[4], delta[5]));
            if (magnitude < 1e-6) {
                break;
            }
        }
        stage_index++;
    }
    output.transform = transform;
    return output;
}

#endif

#if defined(REALSENSE_FOUNDATION_STEREO_ENABLED) && defined(REALSENSE_DIRECT_ENABLED) && defined(REALSENSE_NATIVE_CALIBRATION_ENABLED)

class CalibrationOrt {
public:
    ~CalibrationOrt() {
        if (api && env) {
            api->ReleaseEnv(env);
        }
#if defined(_WIN32)
        if (module) {
            FreeLibrary(module);
        }
#else
        if (module) {
            dlclose(module);
        }
#endif
    }

    bool load(std::string &r_error) {
        if (api && env) {
            return true;
        }
#if defined(_WIN32)
        std::vector<std::filesystem::path> candidates;
        const char *path = std::getenv("ONNXRUNTIME_DLL_PATH");
        if (path && path[0]) {
            candidates.emplace_back(path);
        }
        candidates.emplace_back("native/realsense_shared_memory/bin/onnxruntime.dll");
        candidates.emplace_back("onnxruntime.dll");
        for (const std::filesystem::path &candidate : candidates) {
            std::wstring wide = candidate.wstring();
            module = LoadLibraryW(wide.c_str());
            if (module) {
                break;
            }
        }
        if (!module) {
            r_error = "onnxruntime.dll not found";
            return false;
        }
        auto get_api_base = reinterpret_cast<const OrtApiBase *(ORT_API_CALL *)()>(GetProcAddress(module, "OrtGetApiBase"));
#else
        std::vector<std::filesystem::path> candidates;
        const char *path = std::getenv("ONNXRUNTIME_SO_PATH");
        if (path && path[0]) {
            candidates.emplace_back(path);
        }
        candidates.emplace_back("native/realsense_shared_memory/bin/libonnxruntime.so");
        const char *home = std::getenv("HOME");
        if (home && home[0]) {
            const std::filesystem::path local = std::filesystem::path(home) / ".local/lib";
            if (std::filesystem::is_directory(local)) {
                for (const auto &entry : std::filesystem::directory_iterator(local)) {
                    if (!entry.is_directory() || entry.path().filename().string().rfind("python", 0) != 0) {
                        continue;
                    }
                    const std::filesystem::path capi = entry.path() / "site-packages/onnxruntime/capi";
                    if (std::filesystem::is_directory(capi)) {
                        for (const auto &library : std::filesystem::directory_iterator(capi)) {
                            if (library.path().filename().string().rfind("libonnxruntime.so", 0) == 0) {
                                candidates.push_back(library.path());
                            }
                        }
                    }
                }
            }
        }
        candidates.emplace_back("/usr/lib/x86_64-linux-gnu/libonnxruntime.so");
        candidates.emplace_back("libonnxruntime.so");
        for (const std::filesystem::path &candidate : candidates) {
            module = dlopen(candidate.c_str(), RTLD_NOW | RTLD_LOCAL);
            if (module) {
                break;
            }
        }
        if (!module) {
            const char *error = dlerror();
            r_error = error ? error : "libonnxruntime.so not found";
            return false;
        }
        auto get_api_base = reinterpret_cast<const OrtApiBase *(ORT_API_CALL *)()>(dlsym(module, "OrtGetApiBase"));
#endif
        if (!get_api_base) {
            r_error = "OrtGetApiBase is unavailable";
            return false;
        }
        api = get_api_base()->GetApi(ORT_API_VERSION);
        if (!api) {
            r_error = "ONNX Runtime API version mismatch";
            return false;
        }
        OrtStatus *created = api->CreateEnv(ORT_LOGGING_LEVEL_ERROR, "realsense_pair_calibration", &env);
        if (created) {
            r_error = api->GetErrorMessage(created);
            api->ReleaseStatus(created);
            return false;
        }
        return true;
    }

    bool check(OrtStatus *p_status, std::string &r_error) const {
        if (!p_status) {
            return true;
        }
        r_error = api->GetErrorMessage(p_status);
        api->ReleaseStatus(p_status);
        return false;
    }

#if defined(_WIN32)
    HMODULE module = nullptr;
#else
    void *module = nullptr;
#endif
    const OrtApi *api = nullptr;
    OrtEnv *env = nullptr;
};

bool mask_marker(cv::Mat &p_bgr, int p_marker_id, cv::Mat &r_mask) {
    cv::Matx44d pose;
    double error = 0.0;
    std::vector<cv::Point2f> corners;
    rs2_intrinsics dummy{};
    dummy.fx = 1.0f;
    dummy.fy = 1.0f;
    dummy.ppx = 0.0f;
    dummy.ppy = 0.0f;
    cv::Mat gray;
    cv::cvtColor(p_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    std::vector<std::vector<cv::Point2f>> found_corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(gray, dictionary, found_corners, ids);
    r_mask = cv::Mat::zeros(p_bgr.rows, p_bgr.cols, CV_8U);
    bool found = false;
    for (int index = 0; index < int(ids.size()); ++index) {
        if (ids[index] != p_marker_id) {
            continue;
        }
        cv::Point2f center(0.0f, 0.0f);
        for (const cv::Point2f &point : found_corners[index]) {
            center += point;
        }
        center *= 0.25f;
        std::vector<cv::Point> polygon;
        for (const cv::Point2f &point : found_corners[index]) {
            const cv::Point2f expanded = center + (point - center) * 1.65f;
            polygon.emplace_back(int(std::lround(expanded.x)), int(std::lround(expanded.y)));
        }
        cv::fillConvexPoly(r_mask, polygon, cv::Scalar(255));
        found = true;
    }
    if (!found) {
        return false;
    }
    cv::dilate(r_mask, r_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(31, 31)));
    cv::Scalar mean = cv::mean(p_bgr, 255 - r_mask);
    p_bgr.setTo(mean, r_mask);
    return true;
}

bool run_lightglue(
    const RealSensePairCalibrator::Options &p_options,
    Capture &p_reference,
    Capture &p_target,
    cv::Matx44d &r_target_depth_to_reference_depth,
    int &r_matches,
    int &r_inliers,
    double &r_median,
    std::string &r_solver,
    double &r_reprojection_px,
    std::string &r_error
) {
    CalibrationOrt runtime;
    if (!runtime.load(r_error)) {
        return false;
    }
    if (!std::filesystem::is_regular_file(p_options.model_path)) {
        r_error = "LightGlue ONNX model not found: " + p_options.model_path;
        return false;
    }
    OrtSessionOptions *session_options = nullptr;
    if (!runtime.check(runtime.api->CreateSessionOptions(&session_options), r_error)) {
        return false;
    }
    runtime.check(runtime.api->SetSessionGraphOptimizationLevel(session_options, ORT_ENABLE_ALL), r_error);
    runtime.check(runtime.api->SetIntraOpNumThreads(session_options, 4), r_error);
    runtime.check(runtime.api->SetInterOpNumThreads(session_options, 1), r_error);
    OrtSession *session = nullptr;
#if defined(_WIN32)
    const std::wstring model_path = std::filesystem::path(p_options.model_path).wstring();
#else
    const std::string &model_path = p_options.model_path;
#endif
    if (!runtime.check(runtime.api->CreateSession(runtime.env, model_path.c_str(), session_options, &session), r_error)) {
        runtime.api->ReleaseSessionOptions(session_options);
        return false;
    }
    runtime.api->ReleaseSessionOptions(session_options);
    constexpr int input_width = 960;
    constexpr int input_height = 544;
    cv::Mat target_image = p_target.color_bgr.clone();
    cv::Mat reference_image = p_reference.color_bgr.clone();
    mask_marker(target_image, p_options.marker_id, p_target.marker_mask);
    mask_marker(reference_image, p_options.marker_id, p_reference.marker_mask);
    cv::resize(target_image, target_image, cv::Size(input_width, input_height), 0.0, 0.0, cv::INTER_AREA);
    cv::resize(reference_image, reference_image, cv::Size(input_width, input_height), 0.0, 0.0, cv::INTER_AREA);
    std::vector<float> input(size_t(2 * input_width * input_height));
    const std::array<cv::Mat, 2> images = {target_image, reference_image};
    for (int batch = 0; batch < 2; ++batch) {
        for (int y = 0; y < input_height; ++y) {
            const cv::Vec3b *row = images[batch].ptr<cv::Vec3b>(y);
            for (int x = 0; x < input_width; ++x) {
                const cv::Vec3b bgr = row[x];
                input[size_t(batch * input_width * input_height + y * input_width + x)] =
                    (0.114f * bgr[0] + 0.587f * bgr[1] + 0.299f * bgr[2]) / 255.0f;
            }
        }
    }
    OrtMemoryInfo *memory = nullptr;
    if (!runtime.check(runtime.api->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &memory), r_error)) {
        runtime.api->ReleaseSession(session);
        return false;
    }
    const int64_t input_shape[4] = {2, 1, input_height, input_width};
    OrtValue *input_value = nullptr;
    if (!runtime.check(runtime.api->CreateTensorWithDataAsOrtValue(
        memory,
        input.data(),
        input.size() * sizeof(float),
        input_shape,
        4,
        ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
        &input_value
    ), r_error)) {
        runtime.api->ReleaseMemoryInfo(memory);
        runtime.api->ReleaseSession(session);
        return false;
    }
    const char *input_names[] = {"images"};
    const char *output_names[] = {"keypoints", "matches", "mscores"};
    OrtValue *outputs[3] = {nullptr, nullptr, nullptr};
    const bool ran = runtime.check(runtime.api->Run(
        session, nullptr, input_names, &input_value, 1, output_names, 3, outputs
    ), r_error);
    runtime.api->ReleaseValue(input_value);
    runtime.api->ReleaseMemoryInfo(memory);
    if (!ran) {
        runtime.api->ReleaseSession(session);
        return false;
    }
    int64_t *keypoints = nullptr;
    int64_t *matches = nullptr;
    float *scores = nullptr;
    runtime.check(runtime.api->GetTensorMutableData(outputs[0], reinterpret_cast<void **>(&keypoints)), r_error);
    runtime.check(runtime.api->GetTensorMutableData(outputs[1], reinterpret_cast<void **>(&matches)), r_error);
    runtime.check(runtime.api->GetTensorMutableData(outputs[2], reinterpret_cast<void **>(&scores)), r_error);
    OrtTensorTypeAndShapeInfo *matches_info = nullptr;
    runtime.check(runtime.api->GetTensorTypeAndShape(outputs[1], &matches_info), r_error);
    std::vector<int64_t> matches_shape(2, 0);
    runtime.check(runtime.api->GetDimensions(matches_info, matches_shape.data(), matches_shape.size()), r_error);
    const int match_count = int(matches_shape[0]);
    std::vector<Match3D> lifted;
    lifted.reserve(match_count);
    const double target_scale_x = double(p_target.color_width) / input_width;
    const double target_scale_y = double(p_target.color_height) / input_height;
    const double reference_scale_x = double(p_reference.color_width) / input_width;
    const double reference_scale_y = double(p_reference.color_height) / input_height;
    for (int index = 0; index < match_count; ++index) {
        if (scores[index] < 0.05f) {
            continue;
        }
        const int target_keypoint = int(matches[index * 3 + 1]);
        const int reference_keypoint = int(matches[index * 3 + 2]);
        if (target_keypoint < 0 || target_keypoint >= 1024 || reference_keypoint < 0 || reference_keypoint >= 1024) {
            continue;
        }
        const double target_x = keypoints[(0 * 1024 + target_keypoint) * 2 + 0] * target_scale_x;
        const double target_y = keypoints[(0 * 1024 + target_keypoint) * 2 + 1] * target_scale_y;
        const double reference_x = keypoints[(1 * 1024 + reference_keypoint) * 2 + 0] * reference_scale_x;
        const double reference_y = keypoints[(1 * 1024 + reference_keypoint) * 2 + 1] * reference_scale_y;
        if (
            !p_target.marker_mask.empty() &&
            p_target.marker_mask.at<uint8_t>(std::clamp(int(std::lround(target_y)), 0, p_target.marker_mask.rows - 1), std::clamp(int(std::lround(target_x)), 0, p_target.marker_mask.cols - 1))
        ) {
            continue;
        }
        if (
            !p_reference.marker_mask.empty() &&
            p_reference.marker_mask.at<uint8_t>(std::clamp(int(std::lround(reference_y)), 0, p_reference.marker_mask.rows - 1), std::clamp(int(std::lround(reference_x)), 0, p_reference.marker_mask.cols - 1))
        ) {
            continue;
        }
        const double target_depth = local_depth(
            p_target.aligned_depth_m, target_x, target_y, p_options.min_depth_m, 4.5
        );
        const double reference_depth = local_depth(
            p_reference.aligned_depth_m, reference_x, reference_y, p_options.min_depth_m, 4.5
        );
        if (target_depth <= 0.0 || reference_depth <= 0.0) {
            continue;
        }
        lifted.push_back(Match3D{
            deproject(p_target.color_intrinsics, target_x, target_y, target_depth),
            deproject(p_reference.color_intrinsics, reference_x, reference_y, reference_depth),
            scores[index],
            cv::Point2d(target_x, target_y),
            cv::Point2d(reference_x, reference_y),
        });
    }
    runtime.api->ReleaseTensorTypeAndShapeInfo(matches_info);
    for (OrtValue *output : outputs) {
        runtime.api->ReleaseValue(output);
    }
    runtime.api->ReleaseSession(session);
    balance_match_weights(
        lifted,
        p_target.color_width,
        p_target.color_height,
        p_reference.color_width,
        p_reference.color_height
    );
    const char *match_debug_dir = std::getenv("REALSENSE_CALIBRATION_MATCH_DEBUG_DIR");
    if (match_debug_dir && match_debug_dir[0]) {
        std::filesystem::create_directories(match_debug_dir);
        const std::filesystem::path debug_root(match_debug_dir);
        write_bgr_ppm(debug_root / "reference.ppm", p_reference.color_bgr);
        write_bgr_ppm(debug_root / "target.ppm", p_target.color_bgr);
        std::ofstream matches_file(debug_root / "matches.csv");
        if (matches_file) {
            matches_file << "target_x,target_y,target_z,reference_x,reference_y,reference_z,score\n";
            matches_file << std::setprecision(12);
            for (const Match3D &match : lifted) {
                matches_file << match.target_pixel.x << ',' << match.target_pixel.y << ',' << match.target[2] << ','
                             << match.reference_pixel.x << ',' << match.reference_pixel.y << ',' << match.reference[2] << ','
                             << match.score << '\n';
            }
        }
    }
    r_matches = int(lifted.size());
    cv::Matx44d rigid_target_to_reference;
    std::vector<int> inliers;
    if (!ransac_rigid(lifted, rigid_target_to_reference, inliers, r_median)) {
        r_error = "LightGlue could not find a consistent 3D rigid transform";
        return false;
    }
    r_inliers = int(inliers.size());
    const double inlier_ratio = double(r_inliers) / double(std::max(1, r_matches));
    if (r_inliers < 25 || inlier_ratio < 0.25 || r_median > 0.025) {
        std::ostringstream message;
        message << "LightGlue quality rejected: " << r_inliers << "/" << r_matches
                << " inliers, median=" << r_median << "m";
        r_error = message.str();
        return false;
    }
    struct Candidate {
        std::string name;
        cv::Matx44d transform;
        double reprojection = std::numeric_limits<double>::infinity();
    };
    std::vector<Candidate> candidates;
    candidates.push_back(Candidate{
        "rigid_3d",
        rigid_target_to_reference,
        symmetric_reprojection_error(
            lifted,
            rigid_target_to_reference,
            p_target.color_intrinsics,
            p_reference.color_intrinsics
        ),
    });
    cv::Matx44d forward_pnp;
    cv::Matx44d reverse_pnp;
    int forward_inliers = 0;
    int reverse_inliers = 0;
    const bool have_forward = solve_rgbd_pnp(
        lifted,
        true,
        p_reference.color_intrinsics,
        forward_pnp,
        forward_inliers
    );
    const bool have_reverse = solve_rgbd_pnp(
        lifted,
        false,
        p_target.color_intrinsics,
        reverse_pnp,
        reverse_inliers
    );
    if (have_forward) {
        candidates.push_back(Candidate{
            "target_depth_pnp",
            forward_pnp,
            symmetric_reprojection_error(
                lifted,
                forward_pnp,
                p_target.color_intrinsics,
                p_reference.color_intrinsics
            ),
        });
    }
    if (have_reverse) {
        reverse_pnp = reverse_pnp.inv();
        candidates.push_back(Candidate{
            "reference_depth_pnp",
            reverse_pnp,
            symmetric_reprojection_error(
                lifted,
                reverse_pnp,
                p_target.color_intrinsics,
                p_reference.color_intrinsics
            ),
        });
    }
    if (have_forward && have_reverse) {
        const cv::Matx44d bidirectional = average_transforms({forward_pnp, reverse_pnp});
        candidates.push_back(Candidate{
            "bidirectional_pnp",
            bidirectional,
            symmetric_reprojection_error(
                lifted,
                bidirectional,
                p_target.color_intrinsics,
                p_reference.color_intrinsics
            ),
        });
    }
    cv::Matx44d essential_direction;
    cv::Matx44d essential_median;
    int essential_inliers = 0;
    if (solve_rgb_essential(
        lifted,
        p_target.color_intrinsics,
        p_reference.color_intrinsics,
        essential_direction,
        essential_median,
        essential_inliers
    )) {
        candidates.push_back(Candidate{
            "essential_direction_scale",
            essential_direction,
            symmetric_reprojection_error(
                lifted,
                essential_direction,
                p_target.color_intrinsics,
                p_reference.color_intrinsics
            ),
        });
        candidates.push_back(Candidate{
            "essential_rotation_median_translation",
            essential_median,
            symmetric_reprojection_error(
                lifted,
                essential_median,
                p_target.color_intrinsics,
                p_reference.color_intrinsics
            ),
        });
    }
    if (match_debug_dir && match_debug_dir[0]) {
        for (const Candidate &candidate : candidates) {
            cv::Matx44d depth_transform =
                p_reference.depth_to_color.inv() * candidate.transform * p_target.depth_to_color;
            cv::Matx44d conversion = cv::Matx44d::eye();
            conversion(1, 1) = -1.0;
            conversion(2, 2) = -1.0;
            const cv::Matx44d godot = conversion * depth_transform * conversion;
            std::ostringstream diagnostic;
            diagnostic << "NATIVE_MARKERLESS_CANDIDATE name=" << candidate.name
                       << " reproj=" << candidate.reprojection << " transform=";
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 4; ++col) {
                    if (row != 0 || col != 0) {
                        diagnostic << ",";
                    }
                    diagnostic << std::setprecision(12) << godot(row, col);
                }
            }
            UtilityFunctions::print(String(diagnostic.str().c_str()));
        }
    }
    Candidate selected = candidates.front();
    for (const Candidate &candidate : candidates) {
        const auto delta = transform_delta(rigid_target_to_reference, candidate.transform);
        if (delta.first > 0.08 || delta.second > 8.0) {
            continue;
        }
        if (candidate.reprojection < selected.reprojection) {
            selected = candidate;
        }
    }
    if (!std::isfinite(selected.reprojection) || selected.reprojection > 12.0) {
        r_error = "LightGlue RGB-D pose has excessive symmetric reprojection error";
        return false;
    }
    r_solver = selected.name;
    r_reprojection_px = selected.reprojection;
    r_target_depth_to_reference_depth =
        p_reference.depth_to_color.inv() * selected.transform * p_target.depth_to_color;
    return true;
}

#endif

#if defined(REALSENSE_DIRECT_ENABLED) && defined(REALSENSE_NATIVE_CALIBRATION_ENABLED)
cv::Matx44d cv_to_godot(const cv::Matx44d &p_transform) {
    cv::Matx44d conversion = cv::Matx44d::eye();
    conversion(1, 1) = -1.0;
    conversion(2, 2) = -1.0;
    return conversion * p_transform * conversion;
}
#endif

} // namespace

RealSensePairCalibrator::RealSensePairCalibrator() = default;

RealSensePairCalibrator::~RealSensePairCalibrator() {
    cancel();
    if (worker.joinable()) {
        worker.join();
    }
}

void RealSensePairCalibrator::_bind_methods() {
    ClassDB::bind_method(D_METHOD("start", "options"), &RealSensePairCalibrator::start);
    ClassDB::bind_method(D_METHOD("cancel"), &RealSensePairCalibrator::cancel);
    ClassDB::bind_method(D_METHOD("is_running"), &RealSensePairCalibrator::is_running);
    ClassDB::bind_method(D_METHOD("get_status"), &RealSensePairCalibrator::get_status);
    ClassDB::bind_method(D_METHOD("get_result"), &RealSensePairCalibrator::get_result);
}

bool RealSensePairCalibrator::start(const Dictionary &p_options) {
    if (running.load()) {
        return false;
    }
    if (worker.joinable()) {
        worker.join();
    }
    Options options;
    options.mode = utf8(String(p_options.get("mode", "markerless")));
    options.reference_serial = utf8(String(p_options.get("reference_serial", "")));
    options.target_serial = utf8(String(p_options.get("target_serial", "")));
    options.profile = utf8(String(p_options.get("profile", "highres30")));
    options.model_path = utf8(String(p_options.get("model_path", "")));
    options.onnx_backend = utf8(String(p_options.get("onnx_backend", "onnx_cuda")));
    options.marker_id = int(p_options.get("marker_id", 49));
    options.marker_size_m = double(p_options.get("marker_size_m", 0.15));
    options.capture_frames = std::clamp(int(p_options.get("capture_frames", 10)), 4, 96);
    options.marker_frames = std::clamp(int(p_options.get("marker_frames", 24)), 8, 60);
    options.min_depth_m = std::max(0.05, double(p_options.get("min_depth_m", 0.2)));
    options.max_depth_m = std::max(options.min_depth_m + 0.1, double(p_options.get("max_depth_m", 3.0)));
    PackedFloat64Array initial_values = p_options.get("initial_transform", PackedFloat64Array());
    if (initial_values.size() >= 12) {
        for (int index = 0; index < 12; ++index) {
            options.initial_transform[index] = initial_values[index];
        }
        options.has_initial_transform = true;
    }
    PackedFloat64Array benchmark_values = p_options.get("benchmark_transform", PackedFloat64Array());
    if (benchmark_values.size() >= 12) {
        for (int index = 0; index < 12; ++index) {
            options.benchmark_transform[index] = benchmark_values[index];
        }
        options.has_benchmark_transform = true;
    }
    PackedFloat64Array prior_values = p_options.get("prior_transform", PackedFloat64Array());
    if (prior_values.size() >= 12) {
        for (int index = 0; index < 12; ++index) {
            options.prior_transform[index] = prior_values[index];
        }
        options.has_prior_transform = true;
    }
    if (options.reference_serial.empty() || options.target_serial.empty() || options.reference_serial == options.target_serial) {
        set_status("Native calibration needs two different RealSense serials");
        return false;
    }
    if (options.mode != "aruco" && options.mode != "markerless" && options.mode != "refine") {
        set_status("Native calibration mode must be aruco, markerless, or refine");
        return false;
    }
    if (options.mode == "refine" && !options.has_initial_transform) {
        set_status("Native refine mode needs an initial transform");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        result = Result();
        status = "Native RealSense calibration starting";
    }
    cancel_requested.store(false);
    running.store(true);
    worker = std::thread(&RealSensePairCalibrator::run, this, options);
    return true;
}

void RealSensePairCalibrator::cancel() {
    cancel_requested.store(true);
}

bool RealSensePairCalibrator::is_running() const {
    return running.load();
}

String RealSensePairCalibrator::get_status() const {
    std::lock_guard<std::mutex> lock(state_mutex);
    return String(status.c_str());
}

Dictionary RealSensePairCalibrator::get_result() const {
    std::lock_guard<std::mutex> lock(state_mutex);
    Dictionary payload;
    payload["ready"] = result.ready;
    payload["ok"] = result.ok;
    payload["method"] = String(result.method.c_str());
    payload["status"] = String(result.status.c_str());
    if (!result.ready) {
        return payload;
    }
    Array rotation;
    for (int row = 0; row < 3; ++row) {
        Array values;
        for (int col = 0; col < 3; ++col) {
            values.append(result.rotation[row * 3 + col]);
        }
        rotation.append(values);
    }
    Array translation;
    for (double value : result.translation) {
        translation.append(value);
    }
    payload["R"] = rotation;
    payload["T"] = translation;
    Dictionary details;
    details["matches"] = result.matches;
    details["inliers"] = result.inliers;
    details["match_median_m"] = result.match_median_m;
    details["markerless_solver"] = String(result.markerless_solver.c_str());
    details["markerless_reprojection_px"] = result.markerless_reprojection_px;
    details["icp_fitness"] = result.icp_fitness;
    details["icp_rmse_m"] = result.icp_rmse_m;
    details["refine_translation_m"] = result.refine_translation_m;
    details["refine_rotation_deg"] = result.refine_rotation_deg;
    details["marker_reprojection_px"] = result.marker_reprojection_px;
    details["marker_reference_frames"] = result.marker_reference_frames;
    details["marker_target_frames"] = result.marker_target_frames;
    if (result.has_marker_reference_transform) {
        Array marker_rotation;
        for (int row = 0; row < 3; ++row) {
            Array values;
            for (int col = 0; col < 3; ++col) {
                values.append(result.marker_reference_rotation[row * 3 + col]);
            }
            marker_rotation.append(values);
        }
        Array marker_translation;
        for (double value : result.marker_reference_translation) {
            marker_translation.append(value);
        }
        details["reference_marker_to_depth_godot_R"] = marker_rotation;
        details["reference_marker_to_depth_godot_T"] = marker_translation;
    }
    Array initial_rotation;
    for (int row = 0; row < 3; ++row) {
        Array values;
        for (int col = 0; col < 3; ++col) {
            values.append(result.initial_rotation[row * 3 + col]);
        }
        initial_rotation.append(values);
    }
    Array initial_translation;
    for (double value : result.initial_translation) {
        initial_translation.append(value);
    }
    details["initial_R"] = initial_rotation;
    details["initial_T"] = initial_translation;
    details["initial_overlap_score"] = result.initial_overlap_score;
    details["refined_overlap_score"] = result.refined_overlap_score;
    details["global_search_score"] = result.global_search_score;
    details["global_overlap_score"] = result.global_overlap_score;
    details["benchmark_overlap_score"] = result.benchmark_overlap_score;
    details["benchmark_translation_error_m"] = result.benchmark_translation_error_m;
    details["benchmark_rotation_error_deg"] = result.benchmark_rotation_error_deg;
    details["prior_overlap_score"] = result.prior_overlap_score;
    details["refinement_selected"] = result.refinement_selected;
    details["global_search_selected"] = result.global_search_selected;
    details["prior_selected"] = result.prior_selected;
    payload["details"] = details;
    return payload;
}

void RealSensePairCalibrator::set_status(const std::string &p_status) {
    std::lock_guard<std::mutex> lock(state_mutex);
    status = p_status;
}

void RealSensePairCalibrator::finish(const Result &p_result) {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        result = p_result;
        status = p_result.status;
    }
    running.store(false);
}

void RealSensePairCalibrator::run(Options p_options) {
    Result output;
    output.ready = true;
    output.method = p_options.mode == "aruco"
        ? "native_aruco"
        : (p_options.mode == "markerless" ? "native_markerless_lightglue" : "native_guarded_refine");
#if !defined(REALSENSE_DIRECT_ENABLED) || !defined(REALSENSE_NATIVE_CALIBRATION_ENABLED)
    output.status = "Native calibration unavailable: rebuild the extension with librealsense and OpenCV development files";
    finish(output);
    return;
#else
    Capture reference;
    Capture target;
    std::string error;
    const bool need_marker = p_options.mode == "aruco";
    set_status("Capturing reference RealSense " + p_options.reference_serial);
    if (!capture_camera(p_options, p_options.reference_serial, need_marker, cancel_requested, reference, error)) {
        output.status = error;
        finish(output);
        return;
    }
    set_status("Capturing target RealSense " + p_options.target_serial);
    if (!capture_camera(p_options, p_options.target_serial, need_marker, cancel_requested, target, error)) {
        output.status = error;
        finish(output);
        return;
    }
    if (cancel_requested.load()) {
        output.status = "Native calibration cancelled";
        finish(output);
        return;
    }

    cv::Matx44d initial = cv::Matx44d::eye();
    if (need_marker) {
        set_status("Averaging ArUco poses in depth-camera coordinates");
        cv::Matx44d marker_to_reference = average_transforms(reference.marker_to_depth);
        cv::Matx44d marker_to_target = average_transforms(target.marker_to_depth);
        initial = marker_to_reference * marker_to_target.inv();
        const cv::Matx44d marker_to_reference_godot = cv_to_godot(marker_to_reference);
        output.has_marker_reference_transform = true;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                output.marker_reference_rotation[row * 3 + col] = marker_to_reference_godot(row, col);
            }
            output.marker_reference_translation[row] = marker_to_reference_godot(row, 3);
        }
        output.marker_reference_frames = int(reference.marker_to_depth.size());
        output.marker_target_frames = int(target.marker_to_depth.size());
        std::vector<double> reprojection = reference.marker_reprojection;
        reprojection.insert(reprojection.end(), target.marker_reprojection.begin(), target.marker_reprojection.end());
        if (!reprojection.empty()) {
            const size_t middle = reprojection.size() / 2;
            std::nth_element(reprojection.begin(), reprojection.begin() + middle, reprojection.end());
            output.marker_reprojection_px = reprojection[middle];
        }
        if (output.marker_reprojection_px > 2.0) {
            output.status = "Native ArUco rejected: median reprojection error exceeds 2 pixels";
            finish(output);
            return;
        }
    } else if (p_options.mode == "markerless") {
#if defined(REALSENSE_FOUNDATION_STEREO_ENABLED)
        set_status("Running native SuperPoint + LightGlue markerless matching");
        if (!run_lightglue(
            p_options,
            reference,
            target,
            initial,
            output.matches,
            output.inliers,
            output.match_median_m,
            output.markerless_solver,
            output.markerless_reprojection_px,
            error
        )) {
            output.status = "Native markerless alignment failed: " + error;
            finish(output);
            return;
        }
#else
        output.status = "Native markerless alignment needs the ONNX Runtime-enabled extension";
        finish(output);
        return;
#endif
    } else {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                initial(row, col) = p_options.initial_transform[row * 4 + col];
            }
            initial(row, 3) = p_options.initial_transform[row * 4 + 3];
        }
        initial = cv_to_godot(initial);
    }

    const cv::Matx44d initial_godot = cv_to_godot(initial);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            output.rotation[row * 3 + col] = initial_godot(row, col);
            output.initial_rotation[row * 3 + col] = initial_godot(row, col);
        }
        output.translation[row] = initial_godot(row, 3);
        output.initial_translation[row] = initial_godot(row, 3);
    }

    set_status("Guarded robust cloud refinement");
    IcpResult icp = robust_icp(
        reference,
        target,
        initial,
        p_options.min_depth_m,
        p_options.max_depth_m,
        cancel_requested
    );
    if (cancel_requested.load()) {
        output.status = "Native calibration cancelled";
        finish(output);
        return;
    }
    const AlignmentScore initial_score = alignment_score(
        reference, target, initial, p_options.min_depth_m, p_options.max_depth_m
    );
    const AlignmentScore refined_score = alignment_score(
        reference, target, icp.transform, p_options.min_depth_m, p_options.max_depth_m
    );
    output.initial_overlap_score = initial_score.score;
    output.refined_overlap_score = refined_score.score;
    cv::Matx44d selected_transform = initial;
    double selected_overlap_score = initial_score.score;
    if (refined_score.score > initial_score.score + 0.001) {
        selected_transform = icp.transform;
        selected_overlap_score = refined_score.score;
        output.refinement_selected = true;
    }
    if (!need_marker) {
        set_status("Bounded color-aware six-axis refinement");
        cv::Matx44d searched = global_axis_refine(
            reference,
            target,
            selected_transform,
            p_options.min_depth_m,
            p_options.max_depth_m,
            cancel_requested,
            output.global_search_score
        );
        if (cancel_requested.load()) {
            output.status = "Native calibration cancelled";
            finish(output);
            return;
        }
        const AlignmentScore searched_score = alignment_score(
            reference, target, searched, p_options.min_depth_m, p_options.max_depth_m
        );
        output.global_overlap_score = searched_score.score;
        if (searched_score.score > selected_overlap_score + 0.0005) {
            selected_transform = searched;
            selected_overlap_score = searched_score.score;
            output.global_search_selected = true;
        }
    }
    if (p_options.has_prior_transform) {
        cv::Matx44d prior_godot = cv::Matx44d::eye();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                prior_godot(row, col) = p_options.prior_transform[row * 4 + col];
            }
            prior_godot(row, 3) = p_options.prior_transform[row * 4 + 3];
        }
        const cv::Matx44d prior = cv_to_godot(prior_godot);
        const AlignmentScore prior_score = alignment_score(
            reference, target, prior, p_options.min_depth_m, p_options.max_depth_m
        );
        const auto prior_delta = transform_delta(prior, selected_transform);
        output.prior_overlap_score = prior_score.score;
        if (
            prior_delta.first <= 0.04 &&
            prior_delta.second <= 4.0 &&
            prior_score.score >= selected_overlap_score - 0.004
        ) {
            selected_transform = prior;
            selected_overlap_score = prior_score.score;
            output.prior_selected = true;
        }
    }
    if (p_options.has_benchmark_transform) {
        cv::Matx44d benchmark_godot = cv::Matx44d::eye();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                benchmark_godot(row, col) = p_options.benchmark_transform[row * 4 + col];
            }
            benchmark_godot(row, 3) = p_options.benchmark_transform[row * 4 + 3];
        }
        const cv::Matx44d benchmark = cv_to_godot(benchmark_godot);
        const AlignmentScore benchmark_score = alignment_score(
            reference, target, benchmark, p_options.min_depth_m, p_options.max_depth_m
        );
        const auto benchmark_error = transform_delta(benchmark, selected_transform);
        output.benchmark_overlap_score = benchmark_score.score;
        output.benchmark_translation_error_m = benchmark_error.first;
        output.benchmark_rotation_error_deg = benchmark_error.second;
    }
    const auto refinement = transform_delta(initial, selected_transform);
    output.icp_fitness = icp.fitness;
    output.icp_rmse_m = icp.rmse;
    output.refine_translation_m = refinement.first;
    output.refine_rotation_deg = refinement.second;
    const double max_translation = need_marker ? 0.06 : 0.08;
    const double max_rotation = need_marker ? 4.0 : 5.0;
    if (
        icp.correspondences < 60 ||
        icp.fitness < 0.04 ||
        !std::isfinite(icp.rmse) ||
        icp.rmse > 0.03 ||
        refinement.first > max_translation ||
        refinement.second > max_rotation
    ) {
        std::ostringstream message;
        message << "Native calibration rejected refinement: fitness=" << icp.fitness
                << " rmse=" << icp.rmse << " delta=" << refinement.first << "m/"
                << refinement.second << "deg";
        output.status = message.str();
        finish(output);
        return;
    }
    cv::Matx44d godot = cv_to_godot(selected_transform);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            output.rotation[row * 3 + col] = godot(row, col);
        }
        output.translation[row] = godot(row, 3);
    }
    output.ok = true;
    std::ostringstream message;
    const char *label = need_marker
        ? "Native ArUco"
        : (p_options.mode == "markerless" ? "Native markerless" : "Native guarded refine");
    message << label << " alignment accepted"
            << " | fit=" << icp.fitness << " rmse=" << icp.rmse
            << " refine=" << refinement.first << "m/" << refinement.second << "deg"
            << " overlap=" << initial_score.score << "->" << refined_score.score
            << (output.refinement_selected ? " selected" : " declined");
    if (!need_marker) {
        message << " search=" << output.global_overlap_score
                << (output.global_search_selected ? " selected" : " declined");
    }
    if (p_options.has_prior_transform) {
        message << " prior=" << output.prior_overlap_score
                << (output.prior_selected ? " retained" : " released");
    }
    if (p_options.has_benchmark_transform) {
        message << " truth=" << output.benchmark_overlap_score
                << " error=" << output.benchmark_translation_error_m << "m/"
                << output.benchmark_rotation_error_deg << "deg";
    }
    if (p_options.mode == "markerless") {
        message << " matches=" << output.inliers << "/" << output.matches
                << " solver=" << output.markerless_solver
                << " reproj=" << output.markerless_reprojection_px << "px";
    }
    output.status = message.str();
    finish(output);
#endif
}
