#include "realsense_shared_memory_point_cloud.h"

#include <godot_cpp/classes/rd_shader_source.hpp>
#include <godot_cpp/classes/rd_shader_spirv.hpp>
#include <godot_cpp/classes/rd_uniform.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/rendering_device.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/rect2i.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <future>
#include <memory>
#include <vector>

#ifndef _WIN32
#include <csetjmp>
#include <cstdlib>
extern "C" {
#include <jpeglib.h>
#include <zlib.h>
}
#endif

using namespace godot;

#ifndef _WIN32
struct RgbdJpegError {
	jpeg_error_mgr base;
	jmp_buf recovery;
};

static void rgbd_jpeg_error_exit(j_common_ptr p_info) {
	auto *error = reinterpret_cast<RgbdJpegError *>(p_info->err);
	longjmp(error->recovery, 1);
}

static PackedByteArray encode_rgb_jpeg_fast(
		const PackedByteArray &p_rgb, int p_width, int p_height, int p_quality) {
	PackedByteArray result;
	if (p_width <= 0 || p_height <= 0 || p_rgb.size() < p_width * p_height * 3) {
		return result;
	}
	jpeg_compress_struct encoder{};
	RgbdJpegError error{};
	encoder.err = jpeg_std_error(&error.base);
	error.base.error_exit = rgbd_jpeg_error_exit;
	unsigned char *output = nullptr;
	unsigned long output_size = 0;
	if (setjmp(error.recovery)) {
		jpeg_destroy_compress(&encoder);
		std::free(output);
		return result;
	}
	jpeg_create_compress(&encoder);
	jpeg_mem_dest(&encoder, &output, &output_size);
	encoder.image_width = p_width;
	encoder.image_height = p_height;
	encoder.input_components = 3;
	encoder.in_color_space = JCS_RGB;
	jpeg_set_defaults(&encoder);
	jpeg_set_quality(&encoder, std::clamp(p_quality, 1, 100), TRUE);
	encoder.dct_method = JDCT_FASTEST;
	jpeg_start_compress(&encoder, TRUE);
	const uint8_t *pixels = p_rgb.ptr();
	while (encoder.next_scanline < encoder.image_height) {
		JSAMPROW row = const_cast<JSAMPROW>(pixels + encoder.next_scanline * p_width * 3);
		jpeg_write_scanlines(&encoder, &row, 1);
	}
	jpeg_finish_compress(&encoder);
	result.resize(int64_t(output_size));
	std::memcpy(result.ptrw(), output, output_size);
	jpeg_destroy_compress(&encoder);
	std::free(output);
	return result;
}

static int rgbd_color_channels(Image::Format p_format) {
	switch (p_format) {
		case Image::FORMAT_RGB8:
			return 3;
		case Image::FORMAT_RGBA8:
			return 4;
		default:
			return 0;
	}
}

static PackedByteArray encode_resampled_rgb_jpeg_fast(
		const PackedByteArray &p_source,
		int p_source_width,
		int p_source_height,
		int p_source_channels,
		int p_crop_left,
		int p_crop_top,
		int p_crop_right,
		int p_crop_bottom,
		int p_target_width,
		int p_target_height,
		int p_quality,
		bool p_color_match_enabled,
		const Vector3 &p_color_gain,
		const Vector3 &p_color_bias) {
	PackedByteArray result;
	if (p_source_width <= 0 || p_source_height <= 0 ||
			(p_source_channels != 3 && p_source_channels != 4) ||
			p_crop_left < 0 || p_crop_top < 0 ||
			p_crop_right <= p_crop_left || p_crop_right > p_source_width ||
			p_crop_bottom <= p_crop_top || p_crop_bottom > p_source_height ||
			p_target_width <= 0 || p_target_height <= 0 ||
			p_source.size() < int64_t(p_source_width) * p_source_height * p_source_channels) {
		return result;
	}

	jpeg_compress_struct encoder{};
	RgbdJpegError error{};
	encoder.err = jpeg_std_error(&error.base);
	error.base.error_exit = rgbd_jpeg_error_exit;
	unsigned char *output = nullptr;
	unsigned long output_size = 0;
	if (setjmp(error.recovery)) {
		jpeg_destroy_compress(&encoder);
		std::free(output);
		return result;
	}
	jpeg_create_compress(&encoder);
	jpeg_mem_dest(&encoder, &output, &output_size);
	encoder.image_width = p_target_width;
	encoder.image_height = p_target_height;
	encoder.input_components = 3;
	encoder.in_color_space = JCS_RGB;
	jpeg_set_defaults(&encoder);
	jpeg_set_quality(&encoder, std::clamp(p_quality, 1, 100), TRUE);
	encoder.dct_method = JDCT_FASTEST;
	jpeg_start_compress(&encoder, TRUE);

	const int crop_width = p_crop_right - p_crop_left;
	const int crop_height = p_crop_bottom - p_crop_top;
	std::vector<int> source_x0(p_target_width);
	std::vector<int> source_x1(p_target_width);
	std::vector<float> source_x_mix(p_target_width);
	for (int x = 0; x < p_target_width; ++x) {
		const double source_x = double(p_crop_left) +
				(double(x) + 0.5) * double(crop_width) / double(p_target_width) - 0.5;
		const int x0 = std::clamp(int(std::floor(source_x)), p_crop_left, p_crop_right - 1);
		source_x0[x] = x0;
		source_x1[x] = std::min(p_crop_right - 1, x0 + 1);
		source_x_mix[x] = float(std::clamp(source_x - double(x0), 0.0, 1.0));
	}

	const uint8_t *source = p_source.ptr();
	std::vector<uint8_t> row(int64_t(p_target_width) * 3);
	const float gain[3] = {p_color_gain.x, p_color_gain.y, p_color_gain.z};
	const float bias[3] = {p_color_bias.x, p_color_bias.y, p_color_bias.z};
	while (encoder.next_scanline < encoder.image_height) {
		const int target_y = int(encoder.next_scanline);
		const double source_y = double(p_crop_top) +
				(double(target_y) + 0.5) * double(crop_height) / double(p_target_height) - 0.5;
		const int y0 = std::clamp(int(std::floor(source_y)), p_crop_top, p_crop_bottom - 1);
		const int y1 = std::min(p_crop_bottom - 1, y0 + 1);
		const float y_mix = float(std::clamp(source_y - double(y0), 0.0, 1.0));
		for (int x = 0; x < p_target_width; ++x) {
			const int x0 = source_x0[x];
			const int x1 = source_x1[x];
			const float x_mix = source_x_mix[x];
			const int offsets[4] = {
				(y0 * p_source_width + x0) * p_source_channels,
				(y0 * p_source_width + x1) * p_source_channels,
				(y1 * p_source_width + x0) * p_source_channels,
				(y1 * p_source_width + x1) * p_source_channels,
			};
			for (int channel = 0; channel < 3; ++channel) {
				const float top = float(source[offsets[0] + channel]) +
						(float(source[offsets[1] + channel]) - float(source[offsets[0] + channel])) * x_mix;
				const float bottom = float(source[offsets[2] + channel]) +
						(float(source[offsets[3] + channel]) - float(source[offsets[2] + channel])) * x_mix;
				float value = top + (bottom - top) * y_mix;
				if (p_color_match_enabled) {
					value = std::clamp((value / 255.0f) * gain[channel] + bias[channel], 0.0f, 1.0f) * 255.0f;
				}
				row[x * 3 + channel] = uint8_t(std::clamp(int(std::lround(value)), 0, 255));
			}
		}
		JSAMPROW row_pointer = row.data();
		jpeg_write_scanlines(&encoder, &row_pointer, 1);
	}
	jpeg_finish_compress(&encoder);
	result.resize(int64_t(output_size));
	std::memcpy(result.ptrw(), output, output_size);
	jpeg_destroy_compress(&encoder);
	std::free(output);
	return result;
}

static PackedByteArray deflate_depth_predictive_fast(
		const PackedByteArray &p_depth, int p_width) {
	PackedByteArray result;
	if (p_depth.is_empty() || p_width <= 0 || (p_depth.size() % 2) != 0) {
		return result;
	}
	// RealSense depth is a noisy but locally smooth uint16 millimeter field.
	// Deflating its interleaved low/high bytes wastes most of that structure.
	// Predict from the pixel above, zig-zag the signed modulo delta, then group
	// low and high bytes. The transform is exactly reversible and cuts live
	// D455 depth payloads by roughly 25%, leaving room for more samples at the
	// same tunnel bitrate.
	const int64_t sample_count = p_depth.size() / 2;
	PackedByteArray predicted;
	predicted.resize(p_depth.size());
	const uint8_t *source = p_depth.ptr();
	uint8_t *low_bytes = predicted.ptrw();
	uint8_t *high_bytes = low_bytes + sample_count;
	for (int64_t index = 0; index < sample_count; ++index) {
		const uint16_t current = uint16_t(source[index * 2]) |
				(uint16_t(source[index * 2 + 1]) << 8);
		uint16_t previous = 0;
		if (index >= p_width) {
			const int64_t previous_offset = (index - p_width) * 2;
			previous = uint16_t(source[previous_offset]) |
					(uint16_t(source[previous_offset + 1]) << 8);
		}
		const int16_t delta = static_cast<int16_t>(uint16_t(current - previous));
		const uint16_t zigzag = delta >= 0
				? uint16_t(uint32_t(delta) * 2U)
				: uint16_t(uint32_t(-int32_t(delta)) * 2U - 1U);
		low_bytes[index] = uint8_t(zigzag & 0xff);
		high_bytes[index] = uint8_t((zigzag >> 8) & 0xff);
	}

	uLongf output_size = compressBound(uLong(predicted.size()));
	result.resize(int64_t(output_size));
	const int status = compress2(
		result.ptrw(), &output_size, predicted.ptr(), uLong(predicted.size()), Z_BEST_SPEED
	);
	if (status != Z_OK) {
		result.clear();
		return result;
	}
	result.resize(int64_t(output_size));
	return result;
}

class RgbdDepthCompressionPool {
	struct Task {
		PackedByteArray depth;
		int width = 0;
		std::promise<PackedByteArray> completion;
	};

	std::mutex mutex;
	std::condition_variable condition;
	std::deque<std::shared_ptr<Task>> tasks;
	std::vector<std::thread> workers;
	bool stopping = false;

	void worker_loop() {
		while (true) {
			std::shared_ptr<Task> task;
			{
				std::unique_lock<std::mutex> lock(mutex);
				condition.wait(lock, [this]() {
					return stopping || !tasks.empty();
				});
				if (stopping && tasks.empty()) {
					return;
				}
				task = std::move(tasks.front());
				tasks.pop_front();
			}
			try {
				task->completion.set_value(deflate_depth_predictive_fast(task->depth, task->width));
			} catch (...) {
				task->completion.set_exception(std::current_exception());
			}
		}
	}

public:
	RgbdDepthCompressionPool() {
		workers.emplace_back(&RgbdDepthCompressionPool::worker_loop, this);
		workers.emplace_back(&RgbdDepthCompressionPool::worker_loop, this);
	}

	~RgbdDepthCompressionPool() {
		{
			std::lock_guard<std::mutex> lock(mutex);
			stopping = true;
		}
		condition.notify_all();
		for (std::thread &worker : workers) {
			if (worker.joinable()) {
				worker.join();
			}
		}
	}

	std::future<PackedByteArray> submit(PackedByteArray p_depth, int p_width) {
		auto task = std::make_shared<Task>();
		task->depth = std::move(p_depth);
		task->width = p_width;
		std::future<PackedByteArray> future = task->completion.get_future();
		{
			std::lock_guard<std::mutex> lock(mutex);
			tasks.push_back(std::move(task));
		}
		condition.notify_one();
		return future;
	}
};

static RgbdDepthCompressionPool &rgbd_depth_compression_pool() {
	static RgbdDepthCompressionPool pool;
	return pool;
}
#endif

static float nearest_valid_depth_in_region(
		const PackedByteArray &p_data,
		int p_width,
		int p_height,
		int p_x0,
		int p_y0,
		int p_x1,
		int p_y1,
		float p_min_depth,
		float p_max_depth) {
	if (p_width <= 0 || p_height <= 0 || p_data.size() < p_width * p_height * 4) {
		return 0.0f;
	}
	const int x0 = std::clamp(p_x0, 0, p_width - 1);
	const int y0 = std::clamp(p_y0, 0, p_height - 1);
	const int x1 = std::clamp(p_x1, x0 + 1, p_width);
	const int y1 = std::clamp(p_y1, y0 + 1, p_height);
	// PackedByteArray storage is allocator-aligned. Reading the float image
	// directly avoids hundreds of thousands of bounds-checked decode_float()
	// calls per frame while preserving the exact source samples.
	const float *depths = reinterpret_cast<const float *>(p_data.ptr());
	float nearest = std::numeric_limits<float>::infinity();
	for (int y = y0; y < y1; ++y) {
		for (int x = x0; x < x1; ++x) {
			const float depth = depths[y * p_width + x];
			if (std::isfinite(depth) && depth >= p_min_depth && depth <= p_max_depth) {
				nearest = std::min(nearest, depth);
			}
		}
	}
    if (std::isfinite(nearest)) {
        return nearest;
    }

    // A thin feature can fall exactly between two downsample footprints. Look
    // one source pixel beyond the footprint, but accept the halo only when at
    // least two samples agree closely enough to avoid bridging real edges.
    float halo_nearest = std::numeric_limits<float>::infinity();
    float halo_farthest = 0.0f;
    int halo_count = 0;
    const int halo_x0 = std::max(0, x0 - 1);
    const int halo_y0 = std::max(0, y0 - 1);
    const int halo_x1 = std::min(p_width, x1 + 1);
    const int halo_y1 = std::min(p_height, y1 + 1);
    for (int y = halo_y0; y < halo_y1; ++y) {
        for (int x = halo_x0; x < halo_x1; ++x) {
            if (x >= x0 && x < x1 && y >= y0 && y < y1) {
                continue;
            }
			const float depth = depths[y * p_width + x];
            if (!std::isfinite(depth) || depth < p_min_depth || depth > p_max_depth) {
                continue;
            }
            halo_nearest = std::min(halo_nearest, depth);
            halo_farthest = std::max(halo_farthest, depth);
            ++halo_count;
        }
    }
    if (halo_count >= 2 && std::isfinite(halo_nearest)) {
        const float allowed_spread = std::max(0.04f, halo_nearest * 0.03f);
        if (halo_farthest - halo_nearest <= allowed_spread) {
            return halo_nearest;
        }
    }
    return 0.0f;
}

RealSenseSharedMemoryPointCloud::RealSenseSharedMemoryPointCloud() {
    reader.instantiate();
    secondary_reader.instantiate();
    secondary_transform = Transform3D();
}

RealSenseSharedMemoryPointCloud::~RealSenseSharedMemoryPointCloud() {
    stop_rgbd_encoder_worker();
    release_gpu_mesh_compute_resources();
    if (direct_realsense.is_valid()) {
        direct_realsense->close();
    }
    if (reader.is_valid()) {
        reader->close();
    }
    if (secondary_reader.is_valid()) {
        secondary_reader->close();
    }
}

void RealSenseSharedMemoryPointCloud::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_shared_memory_name", "name"), &RealSenseSharedMemoryPointCloud::set_shared_memory_name);
    ClassDB::bind_method(D_METHOD("get_shared_memory_name"), &RealSenseSharedMemoryPointCloud::get_shared_memory_name);
    ClassDB::bind_method(D_METHOD("set_point_pixel_size", "size"), &RealSenseSharedMemoryPointCloud::set_point_pixel_size);
    ClassDB::bind_method(D_METHOD("get_point_pixel_size"), &RealSenseSharedMemoryPointCloud::get_point_pixel_size);
    ClassDB::bind_method(D_METHOD("set_circular_point_splats", "enabled"), &RealSenseSharedMemoryPointCloud::set_circular_point_splats);
    ClassDB::bind_method(D_METHOD("get_circular_point_splats"), &RealSenseSharedMemoryPointCloud::get_circular_point_splats);
    ClassDB::bind_method(D_METHOD("set_point_cleanup_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_point_cleanup_enabled);
    ClassDB::bind_method(D_METHOD("get_point_cleanup_enabled"), &RealSenseSharedMemoryPointCloud::get_point_cleanup_enabled);
    ClassDB::bind_method(D_METHOD("set_point_cleanup_depth_delta", "delta"), &RealSenseSharedMemoryPointCloud::set_point_cleanup_depth_delta);
    ClassDB::bind_method(D_METHOD("get_point_cleanup_depth_delta"), &RealSenseSharedMemoryPointCloud::get_point_cleanup_depth_delta);
    ClassDB::bind_method(D_METHOD("set_point_cleanup_min_neighbors", "count"), &RealSenseSharedMemoryPointCloud::set_point_cleanup_min_neighbors);
    ClassDB::bind_method(D_METHOD("get_point_cleanup_min_neighbors"), &RealSenseSharedMemoryPointCloud::get_point_cleanup_min_neighbors);
    ClassDB::bind_method(D_METHOD("set_min_depth", "depth"), &RealSenseSharedMemoryPointCloud::set_min_depth);
    ClassDB::bind_method(D_METHOD("get_min_depth"), &RealSenseSharedMemoryPointCloud::get_min_depth);
    ClassDB::bind_method(D_METHOD("set_max_depth", "depth"), &RealSenseSharedMemoryPointCloud::set_max_depth);
    ClassDB::bind_method(D_METHOD("get_max_depth"), &RealSenseSharedMemoryPointCloud::get_max_depth);
    ClassDB::bind_method(D_METHOD("set_render_connected_mesh", "enabled"), &RealSenseSharedMemoryPointCloud::set_render_connected_mesh);
    ClassDB::bind_method(D_METHOD("get_render_connected_mesh"), &RealSenseSharedMemoryPointCloud::get_render_connected_mesh);
    ClassDB::bind_method(D_METHOD("set_gpu_connected_mesh", "enabled"), &RealSenseSharedMemoryPointCloud::set_gpu_connected_mesh);
    ClassDB::bind_method(D_METHOD("get_gpu_connected_mesh"), &RealSenseSharedMemoryPointCloud::get_gpu_connected_mesh);
    ClassDB::bind_method(D_METHOD("set_cpu_project_points", "enabled"), &RealSenseSharedMemoryPointCloud::set_cpu_project_points);
    ClassDB::bind_method(D_METHOD("get_cpu_project_points"), &RealSenseSharedMemoryPointCloud::get_cpu_project_points);
    ClassDB::bind_method(D_METHOD("set_color_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_color_enabled);
    ClassDB::bind_method(D_METHOD("get_color_enabled"), &RealSenseSharedMemoryPointCloud::get_color_enabled);
    ClassDB::bind_method(D_METHOD("set_color_match_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_color_match_enabled);
    ClassDB::bind_method(D_METHOD("get_color_match_enabled"), &RealSenseSharedMemoryPointCloud::get_color_match_enabled);
    ClassDB::bind_method(D_METHOD("set_color_gain", "gain"), &RealSenseSharedMemoryPointCloud::set_color_gain);
    ClassDB::bind_method(D_METHOD("get_color_gain"), &RealSenseSharedMemoryPointCloud::get_color_gain);
    ClassDB::bind_method(D_METHOD("set_color_bias", "bias"), &RealSenseSharedMemoryPointCloud::set_color_bias);
    ClassDB::bind_method(D_METHOD("get_color_bias"), &RealSenseSharedMemoryPointCloud::get_color_bias);
    ClassDB::bind_method(D_METHOD("set_mesh_max_edge", "edge"), &RealSenseSharedMemoryPointCloud::set_mesh_max_edge);
    ClassDB::bind_method(D_METHOD("get_mesh_max_edge"), &RealSenseSharedMemoryPointCloud::get_mesh_max_edge);
    ClassDB::bind_method(D_METHOD("set_mesh_max_depth_delta", "delta"), &RealSenseSharedMemoryPointCloud::set_mesh_max_depth_delta);
    ClassDB::bind_method(D_METHOD("get_mesh_max_depth_delta"), &RealSenseSharedMemoryPointCloud::get_mesh_max_depth_delta);
    ClassDB::bind_method(D_METHOD("set_texture_map_mesh", "enabled"), &RealSenseSharedMemoryPointCloud::set_texture_map_mesh);
    ClassDB::bind_method(D_METHOD("get_texture_map_mesh"), &RealSenseSharedMemoryPointCloud::get_texture_map_mesh);
    ClassDB::bind_method(D_METHOD("set_gpu_mesh_compute_indices", "enabled"), &RealSenseSharedMemoryPointCloud::set_gpu_mesh_compute_indices);
    ClassDB::bind_method(D_METHOD("get_gpu_mesh_compute_indices"), &RealSenseSharedMemoryPointCloud::get_gpu_mesh_compute_indices);
    ClassDB::bind_method(D_METHOD("set_gpu_mesh_static_shader", "enabled"), &RealSenseSharedMemoryPointCloud::set_gpu_mesh_static_shader);
    ClassDB::bind_method(D_METHOD("get_gpu_mesh_static_shader"), &RealSenseSharedMemoryPointCloud::get_gpu_mesh_static_shader);
    ClassDB::bind_method(D_METHOD("set_mesh_min_triangle_area", "area"), &RealSenseSharedMemoryPointCloud::set_mesh_min_triangle_area);
    ClassDB::bind_method(D_METHOD("get_mesh_min_triangle_area"), &RealSenseSharedMemoryPointCloud::get_mesh_min_triangle_area);
    ClassDB::bind_method(D_METHOD("set_mesh_max_color_delta", "delta"), &RealSenseSharedMemoryPointCloud::set_mesh_max_color_delta);
    ClassDB::bind_method(D_METHOD("get_mesh_max_color_delta"), &RealSenseSharedMemoryPointCloud::get_mesh_max_color_delta);
    ClassDB::bind_method(D_METHOD("set_edge_feather_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_edge_feather_enabled);
    ClassDB::bind_method(D_METHOD("get_edge_feather_enabled"), &RealSenseSharedMemoryPointCloud::get_edge_feather_enabled);
    ClassDB::bind_method(D_METHOD("set_edge_feather_width", "width"), &RealSenseSharedMemoryPointCloud::set_edge_feather_width);
    ClassDB::bind_method(D_METHOD("get_edge_feather_width"), &RealSenseSharedMemoryPointCloud::get_edge_feather_width);
    ClassDB::bind_method(D_METHOD("set_edge_feather_min_alpha", "alpha"), &RealSenseSharedMemoryPointCloud::set_edge_feather_min_alpha);
    ClassDB::bind_method(D_METHOD("get_edge_feather_min_alpha"), &RealSenseSharedMemoryPointCloud::get_edge_feather_min_alpha);
    ClassDB::bind_method(D_METHOD("set_delay_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_delay_enabled);
    ClassDB::bind_method(D_METHOD("get_delay_enabled"), &RealSenseSharedMemoryPointCloud::get_delay_enabled);
    ClassDB::bind_method(D_METHOD("set_primary_delay_ms", "delay_ms"), &RealSenseSharedMemoryPointCloud::set_primary_delay_ms);
    ClassDB::bind_method(D_METHOD("get_primary_delay_ms"), &RealSenseSharedMemoryPointCloud::get_primary_delay_ms);
    ClassDB::bind_method(D_METHOD("set_secondary_delay_ms", "delay_ms"), &RealSenseSharedMemoryPointCloud::set_secondary_delay_ms);
    ClassDB::bind_method(D_METHOD("get_secondary_delay_ms"), &RealSenseSharedMemoryPointCloud::get_secondary_delay_ms);
    ClassDB::bind_method(D_METHOD("set_secondary_shared_memory_name", "name"), &RealSenseSharedMemoryPointCloud::set_secondary_shared_memory_name);
    ClassDB::bind_method(D_METHOD("get_secondary_shared_memory_name"), &RealSenseSharedMemoryPointCloud::get_secondary_shared_memory_name);
    ClassDB::bind_method(D_METHOD("set_secondary_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_secondary_enabled);
    ClassDB::bind_method(D_METHOD("get_secondary_enabled"), &RealSenseSharedMemoryPointCloud::get_secondary_enabled);
    ClassDB::bind_method(D_METHOD("set_secondary_transform", "transform"), &RealSenseSharedMemoryPointCloud::set_secondary_transform);
    ClassDB::bind_method(D_METHOD("get_secondary_transform"), &RealSenseSharedMemoryPointCloud::get_secondary_transform);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_enabled);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_enabled"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_enabled);
    ClassDB::bind_method(D_METHOD("restart_direct_realsense"), &RealSenseSharedMemoryPointCloud::restart_direct_realsense);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_serial", "serial"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_serial);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_serial"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_serial);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_stream_profile", "profile"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_stream_profile);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_stream_profile"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_stream_profile);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_depth_source", "source"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_depth_source);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_depth_source"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_depth_source);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_fast_foundation_backend", "backend"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_fast_foundation_backend);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_fast_foundation_backend"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_fast_foundation_backend);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_fast_foundation_profile", "profile"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_fast_foundation_profile);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_fast_foundation_profile"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_fast_foundation_profile);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_fast_foundation_model_path", "path"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_fast_foundation_model_path);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_fast_foundation_model_path"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_fast_foundation_model_path);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_stride", "stride"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_stride);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_stride"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_stride);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_emitter_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_emitter_enabled);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_emitter_enabled"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_emitter_enabled);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_laser_power_percent", "percent"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_laser_power_percent);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_laser_power_percent"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_laser_power_percent);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_crop_rect", "rect"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_crop_rect);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_crop_rect"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_crop_rect);
    ClassDB::bind_method(D_METHOD("set_direct_realsense_filter_config", "config"), &RealSenseSharedMemoryPointCloud::set_direct_realsense_filter_config);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_status"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_status);
    ClassDB::bind_method(D_METHOD("get_direct_realsense_capture_fps"), &RealSenseSharedMemoryPointCloud::get_direct_realsense_capture_fps);
    ClassDB::bind_method(D_METHOD("is_connected"), &RealSenseSharedMemoryPointCloud::is_connected);
    ClassDB::bind_method(D_METHOD("get_render_fps"), &RealSenseSharedMemoryPointCloud::get_render_fps);
    ClassDB::bind_method(D_METHOD("get_display_frame_age_ms"), &RealSenseSharedMemoryPointCloud::get_display_frame_age_ms);
    ClassDB::bind_method(D_METHOD("get_last_triangle_count"), &RealSenseSharedMemoryPointCloud::get_last_triangle_count);
    ClassDB::bind_method(D_METHOD("get_last_point_count"), &RealSenseSharedMemoryPointCloud::get_last_point_count);
    ClassDB::bind_method(D_METHOD("get_depth_image"), &RealSenseSharedMemoryPointCloud::get_depth_image);
    ClassDB::bind_method(D_METHOD("get_color_image"), &RealSenseSharedMemoryPointCloud::get_color_image);
    ClassDB::bind_method(D_METHOD("get_raw_color_image"), &RealSenseSharedMemoryPointCloud::get_raw_color_image);
    ClassDB::bind_method(D_METHOD("get_raw_color_intrinsics"), &RealSenseSharedMemoryPointCloud::get_raw_color_intrinsics);
    ClassDB::bind_method(D_METHOD("get_depth_to_raw_color_extrinsics"), &RealSenseSharedMemoryPointCloud::get_depth_to_raw_color_extrinsics);
    ClassDB::bind_method(
        D_METHOD("get_depth_u16_frame", "max_width"),
        &RealSenseSharedMemoryPointCloud::get_depth_u16_frame,
        DEFVAL(320)
    );
    ClassDB::bind_method(
        D_METHOD("request_rgbd_encode", "max_width", "jpeg_quality"),
        &RealSenseSharedMemoryPointCloud::request_rgbd_encode,
        DEFVAL(320),
        DEFVAL(78)
    );
    ClassDB::bind_method(D_METHOD("take_rgbd_encoded_frame"), &RealSenseSharedMemoryPointCloud::take_rgbd_encoded_frame);
    ClassDB::bind_method(D_METHOD("get_rgbd_encoder_status"), &RealSenseSharedMemoryPointCloud::get_rgbd_encoder_status);
    ClassDB::bind_method(D_METHOD("get_frame_sequence"), &RealSenseSharedMemoryPointCloud::get_frame_sequence);
    ClassDB::bind_method(D_METHOD("get_depth_texture"), &RealSenseSharedMemoryPointCloud::get_depth_texture);
    ClassDB::bind_method(D_METHOD("get_color_texture"), &RealSenseSharedMemoryPointCloud::get_color_texture);
    ClassDB::bind_method(D_METHOD("get_current_intrinsics"), &RealSenseSharedMemoryPointCloud::get_current_intrinsics);
    ClassDB::bind_method(D_METHOD("get_current_grid_size"), &RealSenseSharedMemoryPointCloud::get_current_grid_size);
    ClassDB::bind_method(D_METHOD("get_current_grid_stride"), &RealSenseSharedMemoryPointCloud::get_current_grid_stride);
    ClassDB::bind_method(
        D_METHOD("pick_world_point_near_ray", "ray_origin", "ray_direction", "radius_per_meter", "sample_step"),
        &RealSenseSharedMemoryPointCloud::pick_world_point_near_ray,
        DEFVAL(0.006),
        DEFVAL(2)
    );
    ClassDB::bind_method(D_METHOD("set_fusion_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_fusion_enabled);
    ClassDB::bind_method(D_METHOD("set_fusion_color_only", "enabled"), &RealSenseSharedMemoryPointCloud::set_fusion_color_only);
    ClassDB::bind_method(D_METHOD("set_fusion_peer_depth_texture", "texture"), &RealSenseSharedMemoryPointCloud::set_fusion_peer_depth_texture);
    ClassDB::bind_method(D_METHOD("set_fusion_peer_color_texture", "texture"), &RealSenseSharedMemoryPointCloud::set_fusion_peer_color_texture);
    ClassDB::bind_method(D_METHOD("set_fusion_local_to_peer", "transform"), &RealSenseSharedMemoryPointCloud::set_fusion_local_to_peer);
    ClassDB::bind_method(D_METHOD("set_fusion_peer_intrinsics", "intrinsics"), &RealSenseSharedMemoryPointCloud::set_fusion_peer_intrinsics);
    ClassDB::bind_method(D_METHOD("set_fusion_peer_grid_size", "size"), &RealSenseSharedMemoryPointCloud::set_fusion_peer_grid_size);
    ClassDB::bind_method(D_METHOD("set_fusion_peer_grid_stride", "stride"), &RealSenseSharedMemoryPointCloud::set_fusion_peer_grid_stride);
    ClassDB::bind_method(D_METHOD("set_fusion_priority", "priority"), &RealSenseSharedMemoryPointCloud::set_fusion_priority);
    ClassDB::bind_method(D_METHOD("set_fusion_depth_tolerance", "tolerance"), &RealSenseSharedMemoryPointCloud::set_fusion_depth_tolerance);
    ClassDB::bind_method(D_METHOD("set_robot_mask_capsules", "capsules"), &RealSenseSharedMemoryPointCloud::set_robot_mask_capsules);
    ClassDB::bind_method(D_METHOD("set_robot_mask_enabled", "enabled"), &RealSenseSharedMemoryPointCloud::set_robot_mask_enabled);

    ADD_PROPERTY(PropertyInfo(Variant::STRING, "shared_memory_name"), "set_shared_memory_name", "get_shared_memory_name");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "point_pixel_size"), "set_point_pixel_size", "get_point_pixel_size");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "circular_point_splats"), "set_circular_point_splats", "get_circular_point_splats");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "point_cleanup_enabled"), "set_point_cleanup_enabled", "get_point_cleanup_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "point_cleanup_depth_delta"), "set_point_cleanup_depth_delta", "get_point_cleanup_depth_delta");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "point_cleanup_min_neighbors"), "set_point_cleanup_min_neighbors", "get_point_cleanup_min_neighbors");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_depth"), "set_min_depth", "get_min_depth");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_depth"), "set_max_depth", "get_max_depth");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "render_connected_mesh"), "set_render_connected_mesh", "get_render_connected_mesh");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "gpu_connected_mesh"), "set_gpu_connected_mesh", "get_gpu_connected_mesh");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "cpu_project_points"), "set_cpu_project_points", "get_cpu_project_points");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "color_enabled"), "set_color_enabled", "get_color_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "color_match_enabled"), "set_color_match_enabled", "get_color_match_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "color_gain"), "set_color_gain", "get_color_gain");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "color_bias"), "set_color_bias", "get_color_bias");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mesh_max_edge"), "set_mesh_max_edge", "get_mesh_max_edge");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mesh_max_depth_delta"), "set_mesh_max_depth_delta", "get_mesh_max_depth_delta");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "texture_map_mesh"), "set_texture_map_mesh", "get_texture_map_mesh");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "gpu_mesh_compute_indices"), "set_gpu_mesh_compute_indices", "get_gpu_mesh_compute_indices");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "gpu_mesh_static_shader"), "set_gpu_mesh_static_shader", "get_gpu_mesh_static_shader");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mesh_min_triangle_area"), "set_mesh_min_triangle_area", "get_mesh_min_triangle_area");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mesh_max_color_delta"), "set_mesh_max_color_delta", "get_mesh_max_color_delta");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "edge_feather_enabled"), "set_edge_feather_enabled", "get_edge_feather_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "edge_feather_width"), "set_edge_feather_width", "get_edge_feather_width");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "edge_feather_min_alpha"), "set_edge_feather_min_alpha", "get_edge_feather_min_alpha");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "delay_enabled"), "set_delay_enabled", "get_delay_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "primary_delay_ms"), "set_primary_delay_ms", "get_primary_delay_ms");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "secondary_delay_ms"), "set_secondary_delay_ms", "get_secondary_delay_ms");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "secondary_shared_memory_name"), "set_secondary_shared_memory_name", "get_secondary_shared_memory_name");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "secondary_enabled"), "set_secondary_enabled", "get_secondary_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "direct_realsense_enabled"), "set_direct_realsense_enabled", "get_direct_realsense_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "direct_realsense_serial"), "set_direct_realsense_serial", "get_direct_realsense_serial");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "direct_realsense_stream_profile"), "set_direct_realsense_stream_profile", "get_direct_realsense_stream_profile");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "direct_realsense_depth_source"), "set_direct_realsense_depth_source", "get_direct_realsense_depth_source");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "direct_realsense_fast_foundation_backend"), "set_direct_realsense_fast_foundation_backend", "get_direct_realsense_fast_foundation_backend");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "direct_realsense_fast_foundation_profile"), "set_direct_realsense_fast_foundation_profile", "get_direct_realsense_fast_foundation_profile");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "direct_realsense_fast_foundation_model_path"), "set_direct_realsense_fast_foundation_model_path", "get_direct_realsense_fast_foundation_model_path");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "direct_realsense_stride"), "set_direct_realsense_stride", "get_direct_realsense_stride");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "direct_realsense_emitter_enabled"), "set_direct_realsense_emitter_enabled", "get_direct_realsense_emitter_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "direct_realsense_laser_power_percent", PROPERTY_HINT_RANGE, "0,100,1"), "set_direct_realsense_laser_power_percent", "get_direct_realsense_laser_power_percent");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR4, "direct_realsense_crop_rect"), "set_direct_realsense_crop_rect", "get_direct_realsense_crop_rect");
}

void RealSenseSharedMemoryPointCloud::_ready() {
    ensure_material();
    if (direct_realsense_enabled) {
        if (direct_realsense.is_null()) {
            direct_realsense.instantiate();
        }
        direct_realsense->set_device_serial(direct_realsense_serial);
        direct_realsense->set_stream_profile(direct_realsense_stream_profile);
        direct_realsense->set_depth_source(direct_realsense_depth_source);
        direct_realsense->set_fast_foundation_backend(direct_realsense_fast_foundation_backend);
        direct_realsense->set_fast_foundation_profile(direct_realsense_fast_foundation_profile);
        direct_realsense->set_fast_foundation_model_path(direct_realsense_fast_foundation_model_path);
        direct_realsense->set_stride(direct_realsense_stride);
        direct_realsense->set_emitter_enabled(direct_realsense_emitter_enabled);
        direct_realsense->set_laser_power_percent(float(direct_realsense_laser_power_percent));
        direct_realsense->set_crop_rect(direct_realsense_crop_rect);
        apply_direct_realsense_filter_settings();
    } else if (reader.is_null()) {
        reader.instantiate();
    }
    if (!direct_realsense_enabled && !reader->is_open()) {
        reader->open(shared_memory_name);
    }
    set_process(true);
}

void RealSenseSharedMemoryPointCloud::_process(double p_delta) {
    Ref<Image> depth_image;
    Ref<Image> color_image;
    FrameSnapshot latest_frame;
    const bool cpu_surface_config = cpu_project_points || (render_connected_mesh && !gpu_connected_mesh);
    const bool color_frame_required = color_enabled || cpu_surface_config;
    if (direct_realsense_enabled) {
        if (reader.is_valid() && reader->is_open()) {
            reader->close();
        }
        if (direct_realsense.is_null()) {
            direct_realsense.instantiate();
        }
        direct_realsense->set_device_serial(direct_realsense_serial);
        direct_realsense->set_stream_profile(direct_realsense_stream_profile);
        direct_realsense->set_depth_source(direct_realsense_depth_source);
        direct_realsense->set_fast_foundation_backend(direct_realsense_fast_foundation_backend);
        direct_realsense->set_fast_foundation_profile(direct_realsense_fast_foundation_profile);
        direct_realsense->set_fast_foundation_model_path(direct_realsense_fast_foundation_model_path);
        direct_realsense->set_stride(direct_realsense_stride);
        direct_realsense->set_emitter_enabled(direct_realsense_emitter_enabled);
        direct_realsense->set_laser_power_percent(float(direct_realsense_laser_power_percent));
        direct_realsense->set_crop_rect(direct_realsense_crop_rect);
        direct_realsense->set_color_output_enabled(color_frame_required);
        if (!direct_realsense->is_open()) {
            const double now_sec = now_seconds();
            if (now_sec < next_direct_realsense_open_attempt_sec) {
                direct_realsense_status = direct_realsense->get_status();
                return;
            }
            if (!direct_realsense->open()) {
                next_direct_realsense_open_attempt_sec = now_sec + 1.0;
                direct_realsense_status = direct_realsense->get_status();
                return;
            }
            next_direct_realsense_open_attempt_sec = 0.0;
        }
        if (!direct_realsense->poll()) {
            direct_realsense_status = direct_realsense->get_status();
            return;
        }
        direct_realsense_status = direct_realsense->get_status();
        depth_image = direct_realsense->get_depth_image();
        color_image = direct_realsense->get_color_image();
        latest_frame = make_direct_snapshot(direct_realsense, depth_image, color_image);
    } else {
        if (direct_realsense.is_valid() && direct_realsense->is_open()) {
            direct_realsense->close();
            direct_realsense_status = direct_realsense->get_status();
        }
        if (reader.is_null()) {
            reader.instantiate();
        }
        if (!reader->is_open()) {
            reader->open(shared_memory_name);
            if (!reader->is_open()) {
                return;
            }
        }
        if (!reader->poll()) {
            return;
        }
        depth_image = reader->get_depth_image();
        color_image = reader->get_color_image();
        latest_frame = make_reader_snapshot(reader, depth_image, color_image);
    }
    last_display_frame_seconds = now_seconds();

    int width = latest_frame.width;
    int height = latest_frame.height;
    int stride = latest_frame.stride;
    if (width <= 1 || height <= 1) {
        return;
    }
    current_width = width;
    current_height = height;
    current_stride = stride;
    current_intrinsics = latest_frame.intrinsics;
    const bool cpu_built_surface = cpu_project_points || (render_connected_mesh && !gpu_connected_mesh);
    const bool static_gpu_mesh_surface = render_connected_mesh && gpu_connected_mesh && gpu_mesh_static_shader;
    if (!cpu_built_surface && (!gpu_connected_mesh || static_gpu_mesh_surface) && (width != grid_width || height != grid_height || stride != grid_stride || get_mesh().is_null())) {
        rebuild_mesh(width, height, stride);
    }

    if (depth_image.is_null() || (color_frame_required && color_image.is_null())) {
        return;
    }
    push_frame_history(primary_history, latest_frame);
    FrameSnapshot primary_frame = select_frame_for_delay(primary_history, primary_delay_ms, latest_frame);
    if (primary_frame.depth_image.is_null() || (color_frame_required && primary_frame.color_image.is_null())) {
        return;
    }
    current_width = primary_frame.width;
    current_height = primary_frame.height;
    current_stride = primary_frame.stride;
    current_frame_sequence = primary_frame.sequence;
    current_intrinsics = primary_frame.intrinsics;
    current_depth_image = primary_frame.depth_image;
    current_color_image = primary_frame.color_image;
    if (render_connected_mesh && !gpu_connected_mesh) {
        if (secondary_enabled) {
            if (secondary_reader.is_null()) {
                secondary_reader.instantiate();
            }
            if (!secondary_reader->is_open()) {
                secondary_reader->open(secondary_shared_memory_name);
            }
            if (secondary_reader->is_open()) {
                if (secondary_reader->poll()) {
                    push_reader_frame_history(secondary_history, secondary_reader, secondary_reader->get_depth_image(), secondary_reader->get_color_image());
                }
            }
            last_triangle_count = rebuild_cpu_combined_mesh(primary_frame.depth_image, primary_frame.color_image);
        } else {
            last_triangle_count = rebuild_cpu_connected_mesh(primary_frame.depth_image, primary_frame.color_image);
        }
        frames++;
        fps_accum += p_delta;
        if (fps_accum >= 1.0) {
            render_fps = double(frames) / fps_accum;
            frames = 0;
            fps_accum = 0.0;
        }
        return;
    }
    if (cpu_project_points && !render_connected_mesh) {
        last_triangle_count = 0;
        last_point_count = rebuild_cpu_point_cloud(primary_frame.depth_image, primary_frame.color_image);
        frames++;
        fps_accum += p_delta;
        if (fps_accum >= 1.0) {
            render_fps = double(frames) / fps_accum;
            frames = 0;
            fps_accum = 0.0;
        }
        return;
    }
    if (
        depth_texture.is_null() ||
        depth_texture_width != primary_frame.depth_image->get_width() ||
        depth_texture_height != primary_frame.depth_image->get_height()
    ) {
        depth_texture = ImageTexture::create_from_image(primary_frame.depth_image);
        depth_texture_width = primary_frame.depth_image->get_width();
        depth_texture_height = primary_frame.depth_image->get_height();
    } else {
        depth_texture->update(primary_frame.depth_image);
    }
    if (color_enabled && primary_frame.color_image.is_valid()) {
        if (
            color_texture.is_null() ||
            color_texture_width != primary_frame.color_image->get_width() ||
            color_texture_height != primary_frame.color_image->get_height()
        ) {
            color_texture = ImageTexture::create_from_image(primary_frame.color_image);
            color_texture_width = primary_frame.color_image->get_width();
            color_texture_height = primary_frame.color_image->get_height();
        } else {
            color_texture->update(primary_frame.color_image);
        }
    }
    ensure_material();
    set_material_override(Ref<Material>());
    update_material_params();
    if (render_connected_mesh && gpu_connected_mesh && gpu_mesh_static_shader) {
        if (width != grid_width || height != grid_height || stride != grid_stride || get_mesh().is_null()) {
            rebuild_mesh(width, height, stride);
        }
        last_triangle_count = (width - 1) * (height - 1) * 2;
        last_point_count = width * height;
    } else if (render_connected_mesh && gpu_connected_mesh) {
        last_triangle_count = rebuild_gpu_connected_mesh(primary_frame.depth_image);
        last_point_count = width * height;
    } else {
        last_point_count = width * height;
        last_triangle_count = 0;
    }

    frames++;
    fps_accum += p_delta;
    if (fps_accum >= 1.0) {
        render_fps = double(frames) / fps_accum;
        frames = 0;
        fps_accum = 0.0;
    }
}

void RealSenseSharedMemoryPointCloud::set_shared_memory_name(const String &p_name) {
    if (shared_memory_name == p_name) {
        return;
    }
    shared_memory_name = p_name;
    if (reader.is_null()) {
        reader.instantiate();
    }
    if (reader->is_open()) {
        reader->close();
        reader->open(shared_memory_name);
    }
}

String RealSenseSharedMemoryPointCloud::get_shared_memory_name() const { return shared_memory_name; }

void RealSenseSharedMemoryPointCloud::set_point_pixel_size(double p_size) {
    if (Math::is_equal_approx(point_pixel_size, p_size)) {
        return;
    }
    point_pixel_size = p_size;
    update_material_params();
    if (cpu_project_points) {
        set_mesh(Ref<Mesh>());
    }
}

double RealSenseSharedMemoryPointCloud::get_point_pixel_size() const { return point_pixel_size; }

void RealSenseSharedMemoryPointCloud::set_circular_point_splats(bool p_enabled) {
    if (circular_point_splats == p_enabled) {
        return;
    }
    circular_point_splats = p_enabled;
    update_material_params();
    if (cpu_point_material.is_valid()) {
        cpu_point_material->set_shader_parameter("circular_point_splats", circular_point_splats);
    }
}

bool RealSenseSharedMemoryPointCloud::get_circular_point_splats() const { return circular_point_splats; }

void RealSenseSharedMemoryPointCloud::set_point_cleanup_enabled(bool p_enabled) {
    if (point_cleanup_enabled == p_enabled) {
        return;
    }
    point_cleanup_enabled = p_enabled;
    update_material_params();
    if (cpu_project_points) {
        set_mesh(Ref<Mesh>());
    }
}

bool RealSenseSharedMemoryPointCloud::get_point_cleanup_enabled() const { return point_cleanup_enabled; }

void RealSenseSharedMemoryPointCloud::set_point_cleanup_depth_delta(double p_delta) {
    if (Math::is_equal_approx(point_cleanup_depth_delta, p_delta)) {
        return;
    }
    point_cleanup_depth_delta = MAX(0.001, p_delta);
    update_material_params();
    if (cpu_project_points && point_cleanup_enabled) {
        set_mesh(Ref<Mesh>());
    }
}

double RealSenseSharedMemoryPointCloud::get_point_cleanup_depth_delta() const { return point_cleanup_depth_delta; }

void RealSenseSharedMemoryPointCloud::set_point_cleanup_min_neighbors(double p_count) {
    const double clamped = CLAMP(p_count, 0.0, 4.0);
    if (Math::is_equal_approx(point_cleanup_min_neighbors, clamped)) {
        return;
    }
    point_cleanup_min_neighbors = clamped;
    update_material_params();
    if (cpu_project_points && point_cleanup_enabled) {
        set_mesh(Ref<Mesh>());
    }
}

double RealSenseSharedMemoryPointCloud::get_point_cleanup_min_neighbors() const { return point_cleanup_min_neighbors; }

void RealSenseSharedMemoryPointCloud::set_min_depth(double p_depth) {
    if (Math::is_equal_approx(min_depth, p_depth)) {
        return;
    }
    min_depth = p_depth;
    primary_history.clear();
    set_mesh(Ref<Mesh>());
    update_material_params();
    if (direct_realsense.is_valid()) {
        direct_realsense->set_filter_depth_range(float(min_depth), float(max_depth));
    }
}

double RealSenseSharedMemoryPointCloud::get_min_depth() const { return min_depth; }

void RealSenseSharedMemoryPointCloud::set_max_depth(double p_depth) {
    if (Math::is_equal_approx(max_depth, p_depth)) {
        return;
    }
    max_depth = p_depth;
    primary_history.clear();
    set_mesh(Ref<Mesh>());
    update_material_params();
    if (direct_realsense.is_valid()) {
        direct_realsense->set_filter_depth_range(float(min_depth), float(max_depth));
    }
}

double RealSenseSharedMemoryPointCloud::get_max_depth() const { return max_depth; }

void RealSenseSharedMemoryPointCloud::set_render_connected_mesh(bool p_enabled) {
    if (render_connected_mesh == p_enabled) {
        return;
    }
    render_connected_mesh = p_enabled;
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
}

bool RealSenseSharedMemoryPointCloud::get_render_connected_mesh() const { return render_connected_mesh; }

void RealSenseSharedMemoryPointCloud::set_gpu_connected_mesh(bool p_enabled) {
    if (gpu_connected_mesh == p_enabled) {
        return;
    }
    gpu_connected_mesh = p_enabled;
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
}

bool RealSenseSharedMemoryPointCloud::get_gpu_connected_mesh() const { return gpu_connected_mesh; }

void RealSenseSharedMemoryPointCloud::set_cpu_project_points(bool p_enabled) {
    if (cpu_project_points == p_enabled) {
        return;
    }
    cpu_project_points = p_enabled;
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
}

bool RealSenseSharedMemoryPointCloud::get_cpu_project_points() const { return cpu_project_points; }

void RealSenseSharedMemoryPointCloud::set_color_enabled(bool p_enabled) {
    if (color_enabled == p_enabled) {
        return;
    }
    color_enabled = p_enabled;
    update_material_params();
    if (cpu_point_material.is_valid()) {
        cpu_point_material->set_shader_parameter("color_enabled", color_enabled);
    }
    if (cpu_project_points || (render_connected_mesh && !gpu_connected_mesh)) {
        set_mesh(Ref<Mesh>());
    }
}

bool RealSenseSharedMemoryPointCloud::get_color_enabled() const { return color_enabled; }

void RealSenseSharedMemoryPointCloud::set_color_match_enabled(bool p_enabled) {
    if (color_match_enabled == p_enabled) {
        return;
    }
    color_match_enabled = p_enabled;
    update_material_params();
    if (cpu_project_points || (render_connected_mesh && !gpu_connected_mesh)) {
        set_mesh(Ref<Mesh>());
    }
}

bool RealSenseSharedMemoryPointCloud::get_color_match_enabled() const { return color_match_enabled; }

void RealSenseSharedMemoryPointCloud::set_color_gain(const Vector3 &p_gain) {
    const Vector3 next(
        std::clamp(p_gain.x, 0.0f, 4.0f),
        std::clamp(p_gain.y, 0.0f, 4.0f),
        std::clamp(p_gain.z, 0.0f, 4.0f)
    );
    if (color_gain.is_equal_approx(next)) {
        return;
    }
    color_gain = next;
    update_material_params();
    if (cpu_project_points || (render_connected_mesh && !gpu_connected_mesh)) {
        set_mesh(Ref<Mesh>());
    }
}

Vector3 RealSenseSharedMemoryPointCloud::get_color_gain() const { return color_gain; }

void RealSenseSharedMemoryPointCloud::set_color_bias(const Vector3 &p_bias) {
    const Vector3 next(
        std::clamp(p_bias.x, -1.0f, 1.0f),
        std::clamp(p_bias.y, -1.0f, 1.0f),
        std::clamp(p_bias.z, -1.0f, 1.0f)
    );
    if (color_bias.is_equal_approx(next)) {
        return;
    }
    color_bias = next;
    update_material_params();
    if (cpu_project_points || (render_connected_mesh && !gpu_connected_mesh)) {
        set_mesh(Ref<Mesh>());
    }
}

Vector3 RealSenseSharedMemoryPointCloud::get_color_bias() const { return color_bias; }

void RealSenseSharedMemoryPointCloud::set_mesh_max_edge(double p_edge) {
    if (Math::is_equal_approx(mesh_max_edge, p_edge)) {
        return;
    }
    mesh_max_edge = p_edge;
    set_mesh(Ref<Mesh>());
}

double RealSenseSharedMemoryPointCloud::get_mesh_max_edge() const { return mesh_max_edge; }

void RealSenseSharedMemoryPointCloud::set_mesh_max_depth_delta(double p_delta) {
    if (Math::is_equal_approx(mesh_max_depth_delta, p_delta)) {
        return;
    }
    mesh_max_depth_delta = p_delta;
    set_mesh(Ref<Mesh>());
}

double RealSenseSharedMemoryPointCloud::get_mesh_max_depth_delta() const { return mesh_max_depth_delta; }

void RealSenseSharedMemoryPointCloud::set_texture_map_mesh(bool p_enabled) {
    if (texture_map_mesh == p_enabled) {
        return;
    }
    texture_map_mesh = p_enabled;
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
}

bool RealSenseSharedMemoryPointCloud::get_texture_map_mesh() const { return texture_map_mesh; }

void RealSenseSharedMemoryPointCloud::set_gpu_mesh_compute_indices(bool p_enabled) {
    if (gpu_mesh_compute_indices == p_enabled) {
        return;
    }
    gpu_mesh_compute_indices = p_enabled;
    mesh_compute_failed = false;
    if (!gpu_mesh_compute_indices) {
        release_gpu_mesh_compute_resources();
    }
    if (render_connected_mesh && gpu_connected_mesh) {
        set_mesh(Ref<Mesh>());
    }
}

bool RealSenseSharedMemoryPointCloud::get_gpu_mesh_compute_indices() const { return gpu_mesh_compute_indices; }

void RealSenseSharedMemoryPointCloud::set_gpu_mesh_static_shader(bool p_enabled) {
    if (gpu_mesh_static_shader == p_enabled) {
        return;
    }
    gpu_mesh_static_shader = p_enabled;
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
}

bool RealSenseSharedMemoryPointCloud::get_gpu_mesh_static_shader() const { return gpu_mesh_static_shader; }

void RealSenseSharedMemoryPointCloud::set_mesh_min_triangle_area(double p_area) {
    if (Math::is_equal_approx(mesh_min_triangle_area, p_area)) {
        return;
    }
    mesh_min_triangle_area = p_area;
    set_mesh(Ref<Mesh>());
}

double RealSenseSharedMemoryPointCloud::get_mesh_min_triangle_area() const { return mesh_min_triangle_area; }

void RealSenseSharedMemoryPointCloud::set_mesh_max_color_delta(double p_delta) {
    if (Math::is_equal_approx(mesh_max_color_delta, p_delta)) {
        return;
    }
    mesh_max_color_delta = p_delta;
    set_mesh(Ref<Mesh>());
}

double RealSenseSharedMemoryPointCloud::get_mesh_max_color_delta() const { return mesh_max_color_delta; }

void RealSenseSharedMemoryPointCloud::set_edge_feather_enabled(bool p_enabled) {
    edge_feather_enabled = p_enabled;
    update_material_params();
}

bool RealSenseSharedMemoryPointCloud::get_edge_feather_enabled() const { return edge_feather_enabled; }

void RealSenseSharedMemoryPointCloud::set_edge_feather_width(double p_width) {
    edge_feather_width = std::max(0.001, p_width);
    update_material_params();
}

double RealSenseSharedMemoryPointCloud::get_edge_feather_width() const { return edge_feather_width; }

void RealSenseSharedMemoryPointCloud::set_edge_feather_min_alpha(double p_alpha) {
    edge_feather_min_alpha = std::max(0.0, std::min(1.0, p_alpha));
    update_material_params();
}

double RealSenseSharedMemoryPointCloud::get_edge_feather_min_alpha() const { return edge_feather_min_alpha; }

void RealSenseSharedMemoryPointCloud::set_delay_enabled(bool p_enabled) {
    delay_enabled = p_enabled;
    if (!delay_enabled) {
        primary_history.clear();
        secondary_history.clear();
    }
}

bool RealSenseSharedMemoryPointCloud::get_delay_enabled() const { return delay_enabled; }

void RealSenseSharedMemoryPointCloud::set_primary_delay_ms(double p_delay_ms) {
    primary_delay_ms = std::max(0.0, p_delay_ms);
    primary_history.clear();
}

double RealSenseSharedMemoryPointCloud::get_primary_delay_ms() const { return primary_delay_ms; }

void RealSenseSharedMemoryPointCloud::set_secondary_delay_ms(double p_delay_ms) {
    secondary_delay_ms = std::max(0.0, p_delay_ms);
    secondary_history.clear();
}

double RealSenseSharedMemoryPointCloud::get_secondary_delay_ms() const { return secondary_delay_ms; }

void RealSenseSharedMemoryPointCloud::set_secondary_shared_memory_name(const String &p_name) {
    if (secondary_shared_memory_name == p_name) {
        return;
    }
    secondary_shared_memory_name = p_name;
    if (secondary_reader.is_null()) {
        secondary_reader.instantiate();
    }
    if (secondary_reader->is_open()) {
        secondary_reader->close();
        secondary_reader->open(secondary_shared_memory_name);
    }
    set_mesh(Ref<Mesh>());
}

String RealSenseSharedMemoryPointCloud::get_secondary_shared_memory_name() const { return secondary_shared_memory_name; }

void RealSenseSharedMemoryPointCloud::set_secondary_enabled(bool p_enabled) {
    if (secondary_enabled == p_enabled) {
        return;
    }
    secondary_enabled = p_enabled;
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
}

bool RealSenseSharedMemoryPointCloud::get_secondary_enabled() const { return secondary_enabled; }

void RealSenseSharedMemoryPointCloud::set_secondary_transform(const Transform3D &p_transform) {
    secondary_transform = p_transform;
}

Transform3D RealSenseSharedMemoryPointCloud::get_secondary_transform() const { return secondary_transform; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_enabled(bool p_enabled) {
    if (direct_realsense_enabled == p_enabled) {
        return;
    }
    direct_realsense_enabled = p_enabled;
    primary_history.clear();
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
    current_width = 0;
    current_height = 0;
    current_stride = 1;
    current_intrinsics = Vector4();
    if (direct_realsense_enabled) {
        if (direct_realsense_serial.strip_edges().is_empty()) {
            direct_realsense_enabled = false;
            direct_realsense_status = "RealSense direct capture needs a device serial";
            return;
        }
        if (reader.is_valid() && reader->is_open()) {
            reader->close();
        }
        if (direct_realsense.is_null()) {
            direct_realsense.instantiate();
        }
        direct_realsense->set_device_serial(direct_realsense_serial);
        direct_realsense->set_stream_profile(direct_realsense_stream_profile);
        direct_realsense->set_depth_source(direct_realsense_depth_source);
        direct_realsense->set_fast_foundation_backend(direct_realsense_fast_foundation_backend);
        direct_realsense->set_fast_foundation_profile(direct_realsense_fast_foundation_profile);
        direct_realsense->set_fast_foundation_model_path(direct_realsense_fast_foundation_model_path);
        direct_realsense->set_stride(direct_realsense_stride);
        direct_realsense->set_color_output_enabled(color_enabled || cpu_project_points || (render_connected_mesh && !gpu_connected_mesh));
        apply_direct_realsense_filter_settings();
        next_direct_realsense_open_attempt_sec = 0.0;
        direct_realsense_status = "RealSense direct capture enabled; waiting for process open";
    } else {
        if (direct_realsense.is_valid() && direct_realsense->is_open()) {
            direct_realsense->close();
        }
        direct_realsense_status = direct_realsense.is_valid() ? direct_realsense->get_status() : String("RealSense direct capture is disabled");
        if (reader.is_valid() && !reader->is_open()) {
            reader->open(shared_memory_name);
        }
    }
}

bool RealSenseSharedMemoryPointCloud::get_direct_realsense_enabled() const { return direct_realsense_enabled; }

void RealSenseSharedMemoryPointCloud::restart_direct_realsense() {
    primary_history.clear();
    set_mesh(Ref<Mesh>());
    depth_texture = Ref<ImageTexture>();
    color_texture = Ref<ImageTexture>();
    depth_texture_width = 0;
    depth_texture_height = 0;
    color_texture_width = 0;
    color_texture_height = 0;
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
    current_width = 0;
    current_height = 0;
    current_stride = 1;
    current_intrinsics = Vector4();
    last_display_frame_seconds = 0.0;
    render_fps = 0.0;
    frames = 0;
    fps_accum = 0.0;
    last_triangle_count = 0;
    last_point_count = 0;
    if (direct_realsense.is_valid() && direct_realsense->is_open()) {
        direct_realsense->close();
    }
    next_direct_realsense_open_attempt_sec = 0.0;
    direct_realsense_status = direct_realsense_enabled ? String("RealSense direct capture restart requested") : String("RealSense direct capture is disabled");
}

void RealSenseSharedMemoryPointCloud::set_direct_realsense_serial(const String &p_serial) {
    const String next = p_serial.strip_edges();
    if (direct_realsense_serial == next) {
        return;
    }
    direct_realsense_serial = next;
    primary_history.clear();
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
    current_width = 0;
    current_height = 0;
    current_stride = 1;
    current_intrinsics = Vector4();
    if (direct_realsense.is_valid()) {
        next_direct_realsense_open_attempt_sec = 0.0;
        direct_realsense->set_device_serial(direct_realsense_serial);
        direct_realsense_status = direct_realsense->get_status();
    }
}

String RealSenseSharedMemoryPointCloud::get_direct_realsense_serial() const { return direct_realsense_serial; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_stream_profile(const String &p_profile) {
    String next = p_profile.to_lower();
    if (next != "fast60" && next != "viewer30" && next != "highres30") {
        next = "fast60";
    }
    if (direct_realsense_stream_profile == next) {
        return;
    }
    direct_realsense_stream_profile = next;
    primary_history.clear();
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
    if (direct_realsense.is_valid()) {
        next_direct_realsense_open_attempt_sec = 0.0;
        direct_realsense->set_stream_profile(direct_realsense_stream_profile);
        direct_realsense_status = direct_realsense->get_status();
    }
}

String RealSenseSharedMemoryPointCloud::get_direct_realsense_stream_profile() const { return direct_realsense_stream_profile; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_depth_source(const String &p_source) {
    String next = p_source.to_lower();
    if (next != "sdk_depth" && next != "fast_foundation_native") {
        next = "sdk_depth";
    }
    if (direct_realsense_depth_source == next) {
        return;
    }
    direct_realsense_depth_source = next;
    primary_history.clear();
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
    if (direct_realsense.is_valid()) {
        next_direct_realsense_open_attempt_sec = 0.0;
        direct_realsense->set_depth_source(direct_realsense_depth_source);
        direct_realsense_status = direct_realsense->get_status();
    }
}

String RealSenseSharedMemoryPointCloud::get_direct_realsense_depth_source() const { return direct_realsense_depth_source; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_fast_foundation_backend(const String &p_backend) {
    String next = p_backend.to_lower();
    if (next != "onnx_trt" && next != "onnx_cuda" && next != "onnx_cpu") {
        next = "onnx_cuda";
    }
    if (direct_realsense_fast_foundation_backend == next) {
        return;
    }
    direct_realsense_fast_foundation_backend = next;
    if (direct_realsense.is_valid()) {
        next_direct_realsense_open_attempt_sec = 0.0;
        direct_realsense->set_fast_foundation_backend(direct_realsense_fast_foundation_backend);
        direct_realsense_status = direct_realsense->get_status();
    }
}

String RealSenseSharedMemoryPointCloud::get_direct_realsense_fast_foundation_backend() const { return direct_realsense_fast_foundation_backend; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_fast_foundation_profile(const String &p_profile) {
    String next = p_profile.to_lower();
    if (next != "full_320x736_i4" && next != "rt_256x512_i2" && next != "fast_192x384_i2") {
        next = "fast_192x384_i2";
    }
    if (direct_realsense_fast_foundation_profile == next) {
        return;
    }
    direct_realsense_fast_foundation_profile = next;
    primary_history.clear();
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
    if (direct_realsense.is_valid()) {
        next_direct_realsense_open_attempt_sec = 0.0;
        direct_realsense->set_fast_foundation_profile(direct_realsense_fast_foundation_profile);
        direct_realsense_status = direct_realsense->get_status();
    }
}

String RealSenseSharedMemoryPointCloud::get_direct_realsense_fast_foundation_profile() const { return direct_realsense_fast_foundation_profile; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_fast_foundation_model_path(const String &p_path) {
    if (direct_realsense_fast_foundation_model_path == p_path) {
        return;
    }
    direct_realsense_fast_foundation_model_path = p_path;
    if (direct_realsense.is_valid()) {
        next_direct_realsense_open_attempt_sec = 0.0;
        direct_realsense->set_fast_foundation_model_path(direct_realsense_fast_foundation_model_path);
        direct_realsense_status = direct_realsense->get_status();
    }
}

String RealSenseSharedMemoryPointCloud::get_direct_realsense_fast_foundation_model_path() const { return direct_realsense_fast_foundation_model_path; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_stride(int p_stride) {
    const int next = std::max(1, p_stride);
    if (direct_realsense_stride == next) {
        return;
    }
    direct_realsense_stride = next;
    primary_history.clear();
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
    if (direct_realsense.is_valid()) {
        direct_realsense->set_stride(direct_realsense_stride);
    }
}

int RealSenseSharedMemoryPointCloud::get_direct_realsense_stride() const { return direct_realsense_stride; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_emitter_enabled(bool p_enabled) {
    direct_realsense_emitter_enabled = p_enabled;
    if (direct_realsense.is_valid()) {
        direct_realsense->set_emitter_enabled(p_enabled);
        direct_realsense_status = direct_realsense->get_status();
    }
}

bool RealSenseSharedMemoryPointCloud::get_direct_realsense_emitter_enabled() const { return direct_realsense_emitter_enabled; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_laser_power_percent(double p_percent) {
    direct_realsense_laser_power_percent = std::max(0.0, std::min(100.0, p_percent));
    if (direct_realsense.is_valid()) {
        direct_realsense->set_laser_power_percent(float(direct_realsense_laser_power_percent));
        direct_realsense_status = direct_realsense->get_status();
    }
}

double RealSenseSharedMemoryPointCloud::get_direct_realsense_laser_power_percent() const { return direct_realsense_laser_power_percent; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_crop_rect(const Vector4 &p_rect) {
    direct_realsense_crop_rect = p_rect;
    if (direct_realsense.is_valid()) {
        direct_realsense->set_crop_rect(p_rect);
        direct_realsense_crop_rect = direct_realsense->get_crop_rect();
    }
}

Vector4 RealSenseSharedMemoryPointCloud::get_direct_realsense_crop_rect() const { return direct_realsense_crop_rect; }

void RealSenseSharedMemoryPointCloud::set_direct_realsense_filter_config(const Dictionary &p_config) {
    const bool next_post_processing = p_config.has("post_processing_enabled") ? bool(p_config["post_processing_enabled"]) : direct_realsense_post_processing_enabled;
    const bool next_decimation = p_config.has("decimation_filter_enabled") ? bool(p_config["decimation_filter_enabled"]) : direct_realsense_decimation_filter_enabled;
    const int next_decimation_magnitude = p_config.has("decimation_magnitude") ? std::max(2, std::min(8, int(p_config["decimation_magnitude"]))) : direct_realsense_decimation_magnitude;
    const bool next_rotation = p_config.has("rotation_filter_enabled") ? bool(p_config["rotation_filter_enabled"]) : direct_realsense_rotation_filter_enabled;
    const bool next_hdr_merge = p_config.has("hdr_merge_filter_enabled") ? bool(p_config["hdr_merge_filter_enabled"]) : direct_realsense_hdr_merge_filter_enabled;
    const bool next_sequence_id = p_config.has("sequence_id_filter_enabled") ? bool(p_config["sequence_id_filter_enabled"]) : direct_realsense_sequence_id_filter_enabled;
    const bool next_threshold = p_config.has("threshold_filter_enabled") ? bool(p_config["threshold_filter_enabled"]) : direct_realsense_threshold_filter_enabled;
    const bool next_depth_to_disparity = p_config.has("depth_to_disparity_filter_enabled") ? bool(p_config["depth_to_disparity_filter_enabled"]) : direct_realsense_depth_to_disparity_filter_enabled;
    const bool next_spatial = p_config.has("spatial_filter_enabled") ? bool(p_config["spatial_filter_enabled"]) : direct_realsense_spatial_filter_enabled;
    const bool next_temporal = p_config.has("temporal_filter_enabled") ? bool(p_config["temporal_filter_enabled"]) : direct_realsense_temporal_filter_enabled;
    const bool next_hole_filling = p_config.has("hole_filling_filter_enabled") ? bool(p_config["hole_filling_filter_enabled"]) : direct_realsense_hole_filling_filter_enabled;
    const bool next_disparity_to_depth = p_config.has("disparity_to_depth_filter_enabled") ? bool(p_config["disparity_to_depth_filter_enabled"]) : direct_realsense_disparity_to_depth_filter_enabled;
    const int next_hole_filling_mode = p_config.has("hole_filling_mode") ? std::max(0, std::min(2, int(p_config["hole_filling_mode"]))) : direct_realsense_hole_filling_mode;

    const bool changed =
        direct_realsense_post_processing_enabled != next_post_processing ||
        direct_realsense_decimation_filter_enabled != next_decimation ||
        direct_realsense_decimation_magnitude != next_decimation_magnitude ||
        direct_realsense_rotation_filter_enabled != next_rotation ||
        direct_realsense_hdr_merge_filter_enabled != next_hdr_merge ||
        direct_realsense_sequence_id_filter_enabled != next_sequence_id ||
        direct_realsense_threshold_filter_enabled != next_threshold ||
        direct_realsense_depth_to_disparity_filter_enabled != next_depth_to_disparity ||
        direct_realsense_spatial_filter_enabled != next_spatial ||
        direct_realsense_temporal_filter_enabled != next_temporal ||
        direct_realsense_hole_filling_filter_enabled != next_hole_filling ||
        direct_realsense_disparity_to_depth_filter_enabled != next_disparity_to_depth ||
        direct_realsense_hole_filling_mode != next_hole_filling_mode;

    if (!changed) {
        return;
    }

    direct_realsense_post_processing_enabled = next_post_processing;
    direct_realsense_decimation_filter_enabled = next_decimation;
    direct_realsense_decimation_magnitude = next_decimation_magnitude;
    direct_realsense_rotation_filter_enabled = next_rotation;
    direct_realsense_hdr_merge_filter_enabled = next_hdr_merge;
    direct_realsense_sequence_id_filter_enabled = next_sequence_id;
    direct_realsense_threshold_filter_enabled = next_threshold;
    direct_realsense_depth_to_disparity_filter_enabled = next_depth_to_disparity;
    direct_realsense_spatial_filter_enabled = next_spatial;
    direct_realsense_temporal_filter_enabled = next_temporal;
    direct_realsense_hole_filling_filter_enabled = next_hole_filling;
    direct_realsense_disparity_to_depth_filter_enabled = next_disparity_to_depth;
    direct_realsense_hole_filling_mode = next_hole_filling_mode;

    depth_texture.unref();
    color_texture.unref();
    depth_texture_width = 0;
    depth_texture_height = 0;
    color_texture_width = 0;
    color_texture_height = 0;
    set_mesh(Ref<Mesh>());
    grid_width = 0;
    grid_height = 0;
    grid_stride = 0;
    current_width = 0;
    current_height = 0;
    current_stride = 1;
    current_intrinsics = Vector4();

    if (direct_realsense.is_valid()) {
        direct_realsense->reset_post_processing_filters();
    }
    apply_direct_realsense_filter_settings();
}

String RealSenseSharedMemoryPointCloud::get_direct_realsense_status() const {
    if (direct_realsense.is_valid()) {
        return direct_realsense_status + String("\n") + direct_realsense->get_filter_status();
    }
    return direct_realsense_status;
}

double RealSenseSharedMemoryPointCloud::get_direct_realsense_capture_fps() const {
    if (direct_realsense.is_null()) {
        return 0.0;
    }
    return direct_realsense->get_capture_fps();
}

bool RealSenseSharedMemoryPointCloud::is_connected() const {
    if (direct_realsense_enabled) {
        return direct_realsense.is_valid() && direct_realsense->is_open();
    }
    return reader.is_valid() && reader->is_open();
}

double RealSenseSharedMemoryPointCloud::get_render_fps() const { return render_fps; }

double RealSenseSharedMemoryPointCloud::get_display_frame_age_ms() const {
    if (last_display_frame_seconds <= 0.0) {
        return 0.0;
    }
    return std::max(0.0, (now_seconds() - last_display_frame_seconds) * 1000.0);
}

int RealSenseSharedMemoryPointCloud::get_last_triangle_count() const { return last_triangle_count; }

int RealSenseSharedMemoryPointCloud::get_last_point_count() const { return last_point_count; }

Ref<Image> RealSenseSharedMemoryPointCloud::get_depth_image() const { return current_depth_image; }

Ref<Image> RealSenseSharedMemoryPointCloud::get_color_image() const { return current_color_image; }

Ref<Image> RealSenseSharedMemoryPointCloud::get_raw_color_image() const {
    return direct_realsense.is_valid() ? direct_realsense->get_raw_color_image() : Ref<Image>();
}

Vector4 RealSenseSharedMemoryPointCloud::get_raw_color_intrinsics() const {
    return direct_realsense.is_valid() ? direct_realsense->get_raw_color_intrinsics() : Vector4();
}

PackedFloat32Array RealSenseSharedMemoryPointCloud::get_depth_to_raw_color_extrinsics() const {
    return direct_realsense.is_valid() ? direct_realsense->get_depth_to_raw_color_extrinsics() : PackedFloat32Array();
}

Dictionary RealSenseSharedMemoryPointCloud::get_depth_u16_frame(int p_max_width) const {
    Dictionary result;
    if (current_depth_image.is_null() || current_depth_image->is_empty()) {
        return result;
    }
    const int source_width = current_depth_image->get_width();
    const int source_height = current_depth_image->get_height();
    if (source_width <= 0 || source_height <= 0) {
        return result;
    }
    const int target_width = std::max(1, std::min(source_width, p_max_width));
    const int target_height = std::max(1, int(std::round(double(source_height) * double(target_width) / double(source_width))));
    const PackedByteArray source = current_depth_image->get_data();
    if (source.size() < source_width * source_height * 4) {
        return result;
    }
    PackedByteArray depth_mm;
    depth_mm.resize(target_width * target_height * 2);
    uint8_t *destination = depth_mm.ptrw();
    for (int y = 0; y < target_height; ++y) {
        const int source_y0 = std::min(source_height - 1, int((int64_t(y) * source_height) / target_height));
        const int source_y1 = std::clamp(
            int((int64_t(y + 1) * source_height + target_height - 1) / target_height),
            source_y0 + 1,
            source_height
        );
        for (int x = 0; x < target_width; ++x) {
            const int source_x0 = std::min(source_width - 1, int((int64_t(x) * source_width) / target_width));
            const int source_x1 = std::clamp(
                int((int64_t(x + 1) * source_width + target_width - 1) / target_width),
                source_x0 + 1,
                source_width
            );
            const float depth_m = nearest_valid_depth_in_region(
                source,
                source_width,
                source_height,
                source_x0,
                source_y0,
                source_x1,
                source_y1,
                min_depth,
                max_depth
            );
            uint16_t millimeters = 0;
            if (std::isfinite(depth_m) && depth_m >= min_depth && depth_m <= max_depth) {
                millimeters = uint16_t(std::min(65535.0, std::round(double(depth_m) * 1000.0)));
            }
            const int destination_offset = (y * target_width + x) * 2;
            destination[destination_offset] = uint8_t(millimeters & 0xff);
            destination[destination_offset + 1] = uint8_t((millimeters >> 8) & 0xff);
        }
    }
    const double scale_x = double(target_width) / double(source_width);
    const double scale_y = double(target_height) / double(source_height);
    result["width"] = target_width;
    result["height"] = target_height;
    result["intrinsics"] = Vector4(
        float(current_intrinsics.x * scale_x),
        float(current_intrinsics.y * scale_y),
        float(current_intrinsics.z * scale_x),
        float(current_intrinsics.w * scale_y)
    );
    result["depth_mm"] = depth_mm;
    result["sequence"] = int64_t(current_frame_sequence);
    return result;
}

bool RealSenseSharedMemoryPointCloud::request_rgbd_encode(int p_max_width, int p_jpeg_quality) {
    if (current_depth_image.is_null() || current_depth_image->is_empty() ||
            current_color_image.is_null() || current_color_image->is_empty()) {
        return false;
    }
    if (current_depth_image->get_format() != Image::FORMAT_RF) {
        return false;
    }

    RgbdEncodeJob job;
    job.sequence = current_frame_sequence;
    job.depth_width = current_depth_image->get_width();
    job.depth_height = current_depth_image->get_height();
    job.depth_data = current_depth_image->get_data();
    job.color_width = current_color_image->get_width();
    job.color_height = current_color_image->get_height();
	job.color_format = current_color_image->get_format();
	job.color_data = current_color_image->get_data();
	job.intrinsics = current_intrinsics;
	job.crop_rect = direct_realsense_crop_rect;
	job.min_depth = min_depth;
	job.max_depth = max_depth;
    job.max_width = std::max(1, p_max_width);
    job.jpeg_quality = std::clamp(p_jpeg_quality, 1, 100);
    job.color_match_enabled = color_match_enabled;
    job.color_gain = color_gain;
    job.color_bias = color_bias;
    job.capture_unix_ms = std::chrono::duration<double, std::milli>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    if (job.depth_width <= 0 || job.depth_height <= 0 || job.color_width <= 0 || job.color_height <= 0 ||
            job.depth_data.size() < job.depth_width * job.depth_height * 4 || job.color_data.is_empty()) {
        return false;
    }

    ensure_rgbd_encoder_worker();
    {
        std::lock_guard<std::mutex> lock(rgbd_encoder_mutex);
        if (job.sequence == rgbd_last_submitted_sequence &&
                job.max_width == rgbd_last_submitted_width &&
                job.jpeg_quality == rgbd_last_submitted_quality) {
            return false;
        }
        if (rgbd_encoder_pending_ready) {
            ++rgbd_dropped_pending;
        }
        rgbd_encoder_pending = std::move(job);
        rgbd_encoder_pending_ready = true;
        rgbd_last_submitted_sequence = rgbd_encoder_pending.sequence;
        rgbd_last_submitted_width = rgbd_encoder_pending.max_width;
        rgbd_last_submitted_quality = rgbd_encoder_pending.jpeg_quality;
    }
    rgbd_encoder_cv.notify_one();
    return true;
}

Dictionary RealSenseSharedMemoryPointCloud::take_rgbd_encoded_frame() {
    RgbdEncodeResult completed;
    {
        std::lock_guard<std::mutex> lock(rgbd_encoder_mutex);
        if (!rgbd_encoder_completed_ready) {
            return Dictionary();
        }
        completed = std::move(rgbd_encoder_completed);
        rgbd_encoder_completed = RgbdEncodeResult();
        rgbd_encoder_completed_ready = false;
    }
    if (!completed.valid) {
        return Dictionary();
    }
    Dictionary result;
    result["width"] = completed.width;
    result["height"] = completed.height;
    result["intrinsics"] = completed.intrinsics;
    result["sequence"] = int64_t(completed.sequence);
    result["color_jpeg"] = completed.color_jpeg;
    result["depth_compressed"] = completed.compressed_depth;
#ifdef _WIN32
    result["depth_encoding"] = "u16_mm_deflate";
#else
    result["depth_encoding"] = "u16_mm_vpredict_shuffle_deflate";
#endif
    result["encode_ms"] = completed.encode_ms;
    result["capture_unix_ms"] = completed.capture_unix_ms;
    return result;
}

Dictionary RealSenseSharedMemoryPointCloud::get_rgbd_encoder_status() const {
    std::lock_guard<std::mutex> lock(rgbd_encoder_mutex);
    Dictionary status;
    status["worker_running"] = rgbd_encoder_worker_running;
    status["busy"] = rgbd_encoder_busy;
    status["pending"] = rgbd_encoder_pending_ready;
    status["result_ready"] = rgbd_encoder_completed_ready;
    status["last_submitted_sequence"] = int64_t(rgbd_last_submitted_sequence);
    status["last_completed_sequence"] = int64_t(rgbd_last_completed_sequence);
    status["dropped_pending"] = int64_t(rgbd_dropped_pending);
    status["dropped_completed"] = int64_t(rgbd_dropped_completed);
    return status;
}

void RealSenseSharedMemoryPointCloud::ensure_rgbd_encoder_worker() {
    std::lock_guard<std::mutex> lock(rgbd_encoder_mutex);
    if (rgbd_encoder_worker_running) {
        return;
    }
    rgbd_encoder_stop = false;
    rgbd_encoder_worker_running = true;
    rgbd_encoder_worker_a = std::thread(&RealSenseSharedMemoryPointCloud::rgbd_encoder_worker_loop, this);
    rgbd_encoder_worker_b = std::thread(&RealSenseSharedMemoryPointCloud::rgbd_encoder_worker_loop, this);
}

void RealSenseSharedMemoryPointCloud::stop_rgbd_encoder_worker() {
    {
        std::lock_guard<std::mutex> lock(rgbd_encoder_mutex);
        if (!rgbd_encoder_worker_running &&
                !rgbd_encoder_worker_a.joinable() &&
                !rgbd_encoder_worker_b.joinable()) {
            return;
        }
        rgbd_encoder_stop = true;
        rgbd_encoder_pending_ready = false;
    }
    rgbd_encoder_cv.notify_all();
    if (rgbd_encoder_worker_a.joinable()) {
        rgbd_encoder_worker_a.join();
    }
    if (rgbd_encoder_worker_b.joinable()) {
        rgbd_encoder_worker_b.join();
    }
    std::lock_guard<std::mutex> lock(rgbd_encoder_mutex);
    rgbd_encoder_worker_running = false;
    rgbd_encoder_busy = false;
    rgbd_encoder_active_jobs = 0;
    rgbd_encoder_completed_ready = false;
}

void RealSenseSharedMemoryPointCloud::rgbd_encoder_worker_loop() {
    while (true) {
        RgbdEncodeJob job;
        {
            std::unique_lock<std::mutex> lock(rgbd_encoder_mutex);
            rgbd_encoder_cv.wait(lock, [this]() {
                return rgbd_encoder_stop || rgbd_encoder_pending_ready;
            });
            if (rgbd_encoder_stop) {
                return;
            }
            job = std::move(rgbd_encoder_pending);
            rgbd_encoder_pending = RgbdEncodeJob();
            rgbd_encoder_pending_ready = false;
            ++rgbd_encoder_active_jobs;
            rgbd_encoder_busy = true;
        }

        RgbdEncodeResult completed = encode_rgbd_job(job);
        {
            std::lock_guard<std::mutex> lock(rgbd_encoder_mutex);
            rgbd_encoder_active_jobs = std::max(0, rgbd_encoder_active_jobs - 1);
            rgbd_encoder_busy = rgbd_encoder_active_jobs > 0;
            if (completed.valid) {
                // Workers can finish out of order. Never let an older frame
                // replace a newer completion that is ready for publication.
                const uint64_t newest_sequence = rgbd_encoder_completed_ready
                    ? std::max(rgbd_last_completed_sequence, rgbd_encoder_completed.sequence)
                    : rgbd_last_completed_sequence;
                if (completed.sequence <= newest_sequence) {
                    ++rgbd_dropped_completed;
                    continue;
                }
                if (rgbd_encoder_completed_ready) {
                    ++rgbd_dropped_completed;
                }
                rgbd_last_completed_sequence = completed.sequence;
                rgbd_encoder_completed = std::move(completed);
                rgbd_encoder_completed_ready = true;
            }
        }
    }
}

RealSenseSharedMemoryPointCloud::RgbdEncodeResult RealSenseSharedMemoryPointCloud::encode_rgbd_job(
		const RgbdEncodeJob &p_job) {
	RgbdEncodeResult result;
	const auto started = std::chrono::steady_clock::now();
	if (p_job.depth_width <= 0 || p_job.depth_height <= 0 ||
			p_job.depth_data.size() < p_job.depth_width * p_job.depth_height * 4) {
		return result;
	}
	const double crop_left_normalized = std::clamp(double(p_job.crop_rect.x), 0.0, 0.99);
	const double crop_top_normalized = std::clamp(double(p_job.crop_rect.y), 0.0, 0.99);
	const double crop_right_normalized = std::clamp(double(p_job.crop_rect.z), crop_left_normalized + 0.01, 1.0);
	const double crop_bottom_normalized = std::clamp(double(p_job.crop_rect.w), crop_top_normalized + 0.01, 1.0);
	const int crop_left = std::clamp(int(std::floor(crop_left_normalized * p_job.depth_width)), 0, p_job.depth_width - 1);
	const int crop_top = std::clamp(int(std::floor(crop_top_normalized * p_job.depth_height)), 0, p_job.depth_height - 1);
	const int crop_right = std::clamp(int(std::ceil(crop_right_normalized * p_job.depth_width)), crop_left + 1, p_job.depth_width);
	const int crop_bottom = std::clamp(int(std::ceil(crop_bottom_normalized * p_job.depth_height)), crop_top + 1, p_job.depth_height);
	const int crop_width = crop_right - crop_left;
	const int crop_height = crop_bottom - crop_top;
	const int target_width = std::max(1, std::min(crop_width, p_job.max_width));
	const int target_height = std::max(
		1,
		int(std::round(double(crop_height) * double(target_width) / double(crop_width)))
	);
	PackedByteArray depth_mm;
    depth_mm.resize(target_width * target_height * 2);
    uint8_t *destination = depth_mm.ptrw();
    for (int y = 0; y < target_height; ++y) {
		const int source_y0 = std::min(
			crop_bottom - 1,
			crop_top + int((int64_t(y) * crop_height) / target_height)
		);
		const int source_y1 = std::clamp(
			crop_top + int((int64_t(y + 1) * crop_height + target_height - 1) / target_height),
			source_y0 + 1,
			crop_bottom
		);
        for (int x = 0; x < target_width; ++x) {
			const int source_x0 = std::min(
				crop_right - 1,
				crop_left + int((int64_t(x) * crop_width) / target_width)
			);
			const int source_x1 = std::clamp(
				crop_left + int((int64_t(x + 1) * crop_width + target_width - 1) / target_width),
				source_x0 + 1,
				crop_right
			);
            const float depth_m = nearest_valid_depth_in_region(
				p_job.depth_data,
				p_job.depth_width,
				p_job.depth_height,
				source_x0,
				source_y0,
				source_x1,
				source_y1,
				0.0001f,
				p_job.max_depth
			);
            uint16_t millimeters = 0;
			if (std::isfinite(depth_m) && depth_m > 0.0f && depth_m <= p_job.max_depth) {
                millimeters = uint16_t(std::min(65535.0, std::round(double(depth_m) * 1000.0)));
            }
            const int destination_offset = (y * target_width + x) * 2;
            destination[destination_offset] = uint8_t(millimeters & 0xff);
            destination[destination_offset + 1] = uint8_t((millimeters >> 8) & 0xff);
        }
    }

	const int color_left = std::clamp(int(std::floor(crop_left_normalized * p_job.color_width)), 0, p_job.color_width - 1);
	const int color_top = std::clamp(int(std::floor(crop_top_normalized * p_job.color_height)), 0, p_job.color_height - 1);
	const int color_right = std::clamp(int(std::ceil(crop_right_normalized * p_job.color_width)), color_left + 1, p_job.color_width);
	const int color_bottom = std::clamp(int(std::ceil(crop_bottom_normalized * p_job.color_height)), color_top + 1, p_job.color_height);
    PackedByteArray color_jpeg;
    PackedByteArray compressed_depth;
#ifndef _WIN32
    // Color and depth are independent payloads. A persistent depth pool avoids
    // creating and destroying an OS thread for every synchronized frame.
    auto depth_future = rgbd_depth_compression_pool().submit(depth_mm, target_width);
    const int source_channels = rgbd_color_channels(p_job.color_format);
    if (source_channels > 0) {
        // Resize/crop directly into libjpeg scanlines. This removes the
        // create/get_region/resize/convert chain and its full-frame copies.
        color_jpeg = encode_resampled_rgb_jpeg_fast(
            p_job.color_data,
            p_job.color_width,
            p_job.color_height,
            source_channels,
            color_left,
            color_top,
            color_right,
            color_bottom,
            target_width,
            target_height,
            p_job.jpeg_quality,
            p_job.color_match_enabled,
            p_job.color_gain,
            p_job.color_bias
        );
    }
    // Preserve support for uncommon shared-memory color formats.
    if (color_jpeg.is_empty()) {
        Ref<Image> color_image = Image::create_from_data(
            p_job.color_width,
            p_job.color_height,
            false,
            p_job.color_format,
            p_job.color_data
        );
        if (color_image.is_null() || color_image->is_empty()) {
            return result;
        }
        color_image = color_image->get_region(Rect2i(color_left, color_top, color_right - color_left, color_bottom - color_top));
        if (color_image.is_null() || color_image->is_empty()) {
            return result;
        }
        if (color_image->get_width() != target_width || color_image->get_height() != target_height) {
            color_image->resize(target_width, target_height, Image::INTERPOLATE_BILINEAR);
        }
        if (color_image->get_format() != Image::FORMAT_RGB8) {
            color_image->convert(Image::FORMAT_RGB8);
        }
        if (p_job.color_match_enabled) {
            PackedByteArray corrected = color_image->get_data();
            uint8_t *pixels = corrected.ptrw();
            const int pixel_count = target_width * target_height;
            const float gain[3] = {p_job.color_gain.x, p_job.color_gain.y, p_job.color_gain.z};
            const float bias[3] = {p_job.color_bias.x, p_job.color_bias.y, p_job.color_bias.z};
            for (int pixel = 0; pixel < pixel_count; ++pixel) {
                const int offset = pixel * 3;
                for (int channel = 0; channel < 3; ++channel) {
                    const float value = float(pixels[offset + channel]) / 255.0f;
                    pixels[offset + channel] = uint8_t(std::round(
                        std::clamp(value * gain[channel] + bias[channel], 0.0f, 1.0f) * 255.0f
                    ));
                }
            }
            color_image = Image::create_from_data(target_width, target_height, false, Image::FORMAT_RGB8, corrected);
        }
        color_jpeg = encode_rgb_jpeg_fast(
            color_image->get_data(), target_width, target_height, p_job.jpeg_quality
        );
    }
    compressed_depth = depth_future.get();
#else
    Ref<Image> color_image = Image::create_from_data(
        p_job.color_width,
        p_job.color_height,
        false,
        p_job.color_format,
        p_job.color_data
    );
    if (color_image.is_null() || color_image->is_empty()) {
        return result;
    }
    color_image = color_image->get_region(Rect2i(color_left, color_top, color_right - color_left, color_bottom - color_top));
    if (color_image.is_null() || color_image->is_empty()) {
        return result;
    }
    if (color_image->get_width() != target_width || color_image->get_height() != target_height) {
        color_image->resize(target_width, target_height, Image::INTERPOLATE_BILINEAR);
    }
    if (color_image->get_format() != Image::FORMAT_RGB8) {
        color_image->convert(Image::FORMAT_RGB8);
    }
    if (p_job.color_match_enabled) {
        PackedByteArray corrected = color_image->get_data();
        uint8_t *pixels = corrected.ptrw();
        const int pixel_count = target_width * target_height;
        const float gain[3] = {p_job.color_gain.x, p_job.color_gain.y, p_job.color_gain.z};
        const float bias[3] = {p_job.color_bias.x, p_job.color_bias.y, p_job.color_bias.z};
        for (int pixel = 0; pixel < pixel_count; ++pixel) {
            const int offset = pixel * 3;
            for (int channel = 0; channel < 3; ++channel) {
                const float value = float(pixels[offset + channel]) / 255.0f;
                pixels[offset + channel] = uint8_t(std::round(
                    std::clamp(value * gain[channel] + bias[channel], 0.0f, 1.0f) * 255.0f
                ));
            }
        }
        color_image = Image::create_from_data(target_width, target_height, false, Image::FORMAT_RGB8, corrected);
    }
    color_jpeg = color_image->save_jpg_to_buffer(float(p_job.jpeg_quality) / 100.0f);
    compressed_depth = depth_mm.compress(FileAccess::COMPRESSION_DEFLATE);
#endif
    if (color_jpeg.is_empty() || compressed_depth.is_empty()) {
        return result;
    }

	const double scale_x = double(target_width) / double(crop_width);
	const double scale_y = double(target_height) / double(crop_height);
    result.valid = true;
    result.sequence = p_job.sequence;
    result.width = target_width;
    result.height = target_height;
    result.intrinsics = Vector4(
        float(p_job.intrinsics.x * scale_x),
        float(p_job.intrinsics.y * scale_y),
		float((p_job.intrinsics.z - crop_left) * scale_x),
		float((p_job.intrinsics.w - crop_top) * scale_y)
    );
    result.color_jpeg = std::move(color_jpeg);
    result.compressed_depth = std::move(compressed_depth);
    result.capture_unix_ms = p_job.capture_unix_ms;
    result.encode_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started
    ).count();
    return result;
}

int64_t RealSenseSharedMemoryPointCloud::get_frame_sequence() const { return int64_t(current_frame_sequence); }

Ref<Texture2D> RealSenseSharedMemoryPointCloud::get_depth_texture() const { return depth_texture; }

Ref<Texture2D> RealSenseSharedMemoryPointCloud::get_color_texture() const { return color_texture; }

Vector4 RealSenseSharedMemoryPointCloud::get_current_intrinsics() const { return current_intrinsics; }

Vector2 RealSenseSharedMemoryPointCloud::get_current_grid_size() const {
    return Vector2(current_width, current_height);
}

int RealSenseSharedMemoryPointCloud::get_current_grid_stride() const { return current_stride; }

Dictionary RealSenseSharedMemoryPointCloud::pick_world_point_near_ray(
    const Vector3 &p_ray_origin,
    const Vector3 &p_ray_direction,
    double p_radius_per_meter,
    int p_sample_step
) const {
    Dictionary result;
    result["hit"] = false;
    if (current_depth_image.is_null() || current_depth_image->is_empty()) {
        return result;
    }
    const int width = current_width;
    const int height = current_height;
    const int grid_stride = std::max(1, current_stride);
    const int sample_step = std::clamp(p_sample_step, 1, 16);
    if (width <= 1 || height <= 1) {
        return result;
    }
    const PackedByteArray depth_data = current_depth_image->get_data();
    if (depth_data.size() < width * height * 4) {
        return result;
    }
    Vector3 ray_direction = p_ray_direction.normalized();
    if (!ray_direction.is_finite() || ray_direction.length_squared() < 0.99) {
        return result;
    }
    const double fx = std::max(0.000001, double(current_intrinsics.x));
    const double fy = std::max(0.000001, double(current_intrinsics.y));
    const double ppx = current_intrinsics.z;
    const double ppy = current_intrinsics.w;
    const double angular_radius = std::clamp(p_radius_per_meter, 0.001, 0.05);
    const Transform3D local_to_world = get_global_transform();
    double best_along = std::numeric_limits<double>::infinity();
    double best_ray_distance_sq = std::numeric_limits<double>::infinity();
    Vector3 best_world_point;
    int best_x = -1;
    int best_y = -1;
    for (int y = 0; y < height; y += sample_step) {
        const double py = double(y * grid_stride);
        for (int x = 0; x < width; x += sample_step) {
            const float depth_m = depth_data.decode_float((y * width + x) * 4);
            if (!std::isfinite(depth_m) || depth_m < min_depth || depth_m > max_depth) {
                continue;
            }
            const double px = double(x * grid_stride);
            const Vector3 local_point(
                float((px - ppx) * double(depth_m) / fx),
                float(-(py - ppy) * double(depth_m) / fy),
                -depth_m
            );
            const Vector3 world_point = local_to_world.xform(local_point);
            const Vector3 delta = world_point - p_ray_origin;
            const double along = delta.dot(ray_direction);
            if (along <= 0.0 || along >= best_along) {
                continue;
            }
            const double ray_distance_sq = std::max(0.0, double(delta.length_squared()) - along * along);
            const double acceptance_radius = std::max(0.012, along * angular_radius * double(sample_step));
            if (ray_distance_sq > acceptance_radius * acceptance_radius) {
                continue;
            }
            best_along = along;
            best_ray_distance_sq = ray_distance_sq;
            best_world_point = world_point;
            best_x = x;
            best_y = y;
        }
    }
    if (best_x < 0) {
        return result;
    }
    result["hit"] = true;
    result["position"] = best_world_point;
    result["distance"] = best_along;
    result["ray_distance"] = std::sqrt(best_ray_distance_sq);
    result["pixel"] = Vector2(best_x, best_y);
    return result;
}

void RealSenseSharedMemoryPointCloud::set_fusion_enabled(bool p_enabled) {
    fusion_enabled = p_enabled;
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_fusion_color_only(bool p_enabled) {
    fusion_color_only = p_enabled;
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_fusion_peer_depth_texture(const Ref<Texture2D> &p_texture) {
    fusion_peer_depth_texture = p_texture;
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_fusion_peer_color_texture(const Ref<Texture2D> &p_texture) {
    fusion_peer_color_texture = p_texture;
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_fusion_local_to_peer(const Transform3D &p_transform) {
    fusion_local_to_peer = p_transform;
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_fusion_peer_intrinsics(const Vector4 &p_intrinsics) {
    fusion_peer_intrinsics = p_intrinsics;
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_fusion_peer_grid_size(const Vector2 &p_size) {
    fusion_peer_grid_size = p_size;
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_fusion_peer_grid_stride(int p_stride) {
    fusion_peer_grid_stride = std::max(1, p_stride);
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_fusion_priority(int p_priority) {
    fusion_priority = p_priority;
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_fusion_depth_tolerance(double p_tolerance) {
    fusion_depth_tolerance = std::max(0.001, p_tolerance);
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_robot_mask_capsules(const PackedVector4Array &p_capsules) {
    robot_mask_capsules = p_capsules;
    if (robot_mask_capsules.size() > 12) {
        robot_mask_capsules.resize(12);
    }
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::set_robot_mask_enabled(bool p_enabled) {
    robot_mask_enabled = p_enabled;
    update_material_params();
}

void RealSenseSharedMemoryPointCloud::ensure_material() {
    const bool want_gpu_mesh = render_connected_mesh && gpu_connected_mesh;
    const bool want_static_shader_mesh = want_gpu_mesh && gpu_mesh_static_shader;
    if (
        material.is_valid()
        && material_gpu_mesh_mode == want_gpu_mesh
        && material_texture_map_mode == texture_map_mesh
        && material_static_shader_mesh_mode == want_static_shader_mesh
    ) {
        return;
    }
    Ref<Shader> shader;
    shader.instantiate();
    const String color_sampler = texture_map_mesh
        ? String("uniform sampler2D color_tex : source_color, filter_nearest;")
        : String("uniform sampler2D color_tex : filter_nearest;");
    String shader_code;
    if (want_static_shader_mesh) {
        shader_code = R"(
shader_type spatial;
render_mode unshaded, cull_disabled, depth_draw_opaque;

uniform sampler2D depth_tex : filter_nearest;
__COLOR_TEX_UNIFORM__
uniform vec4 intrinsics = vec4(600.0, 600.0, 320.0, 240.0);
uniform vec2 depth_range = vec2(0.2, 4.5);
uniform vec2 texel_size = vec2(0.01, 0.01);
uniform float grid_stride = 1.0;
uniform float max_depth_delta = 0.05;
uniform float max_mesh_edge = 0.08;
uniform bool point_cleanup_enabled = false;
uniform float point_cleanup_depth_delta = 0.06;
uniform float point_cleanup_min_neighbors = 2.0;
uniform bool color_enabled = true;
uniform bool color_match_enabled = false;
uniform vec3 color_gain = vec3(1.0);
uniform vec3 color_bias = vec3(0.0);
uniform bool color_source_is_linear = true;
uniform bool edge_feather_enabled = false;
uniform float edge_feather_width = 0.035;
uniform float edge_feather_min_alpha = 0.20;
uniform bool fusion_enabled = false;
uniform bool fusion_color_only = false;
uniform sampler2D fusion_peer_depth_tex : filter_nearest;
uniform sampler2D fusion_peer_color_tex : source_color, filter_nearest;
uniform mat4 fusion_local_to_peer;
uniform vec4 fusion_peer_intrinsics = vec4(600.0, 600.0, 320.0, 240.0);
uniform vec2 fusion_peer_grid_size = vec2(1.0, 1.0);
uniform float fusion_peer_grid_stride = 1.0;
uniform float fusion_depth_tolerance = 0.035;
uniform int fusion_priority = 0;
uniform bool robot_mask_enabled = false;
uniform int robot_capsule_count = 0;
uniform vec4 robot_capsules[12];

varying vec2 point_uv;
varying vec2 tri_uv_a;
varying vec2 tri_uv_b;
varying vec2 tri_uv_c;
varying float point_valid;
varying vec3 point_position_local;

vec3 linear_to_srgb(vec3 value) {
    vec3 low = value * 12.92;
    vec3 high = 1.055 * pow(max(value, vec3(0.0)), vec3(1.0 / 2.4)) - 0.055;
    return mix(low, high, step(vec3(0.0031308), value));
}

vec3 srgb_to_linear(vec3 value) {
    vec3 low = value / 12.92;
    vec3 high = pow(max((value + 0.055) / 1.055, vec3(0.0)), vec3(2.4));
    return mix(low, high, step(vec3(0.04045), value));
}

vec3 matched_rgb(vec3 value) {
    if (!color_match_enabled) return value;
    vec3 working = color_source_is_linear ? linear_to_srgb(value) : value;
    working = clamp(working * color_gain + color_bias, vec3(0.0), vec3(1.0));
    return color_source_is_linear ? srgb_to_linear(working) : working;
}

vec3 project_uv(vec2 uv, float depth_m) {
    vec2 grid_size = 1.0 / max(texel_size, vec2(0.000001));
    vec2 pixel = (uv * grid_size - vec2(0.5)) * grid_stride;
    return vec3(
        (pixel.x - intrinsics.z) * depth_m / max(0.000001, intrinsics.x),
        -(pixel.y - intrinsics.w) * depth_m / max(0.000001, intrinsics.y),
        -depth_m
    );
}

bool depth_in_range(float d) {
    return d >= depth_range.x && d <= depth_range.y;
}

float depth_delta_limit(float a, float b) {
    return max_depth_delta * max(max(abs(a), abs(b)), 0.000001);
}

float triangle_delta_limit(float da, float db, float dc) {
    return max_depth_delta * max(max(abs(da), abs(db)), max(abs(dc), 0.000001));
}

bool triangle_depth_ok(float da, float db, float dc) {
    return depth_in_range(da)
        && depth_in_range(db)
        && depth_in_range(dc)
        && abs(da - db) <= depth_delta_limit(da, db)
        && abs(db - dc) <= depth_delta_limit(db, dc)
        && abs(dc - da) <= depth_delta_limit(dc, da);
}

bool triangle_edge_ok(vec3 pa, vec3 pb, vec3 pc) {
    float max_edge_sq = max_mesh_edge * max_mesh_edge;
    float eab = dot(pa - pb, pa - pb);
    float ebc = dot(pb - pc, pb - pc);
    float eca = dot(pc - pa, pc - pa);
    return eab <= max_edge_sq && ebc <= max_edge_sq && eca <= max_edge_sq;
}

float close_neighbor_count_delta(vec2 uv, float d, float delta) {
    float dl = texture(depth_tex, uv + vec2(-texel_size.x, 0.0)).r;
    float dr = texture(depth_tex, uv + vec2(texel_size.x, 0.0)).r;
    float du = texture(depth_tex, uv + vec2(0.0, -texel_size.y)).r;
    float dd = texture(depth_tex, uv + vec2(0.0, texel_size.y)).r;
    float close_neighbors = 0.0;
    close_neighbors += (depth_in_range(dl) && abs(d - dl) <= delta) ? 1.0 : 0.0;
    close_neighbors += (depth_in_range(dr) && abs(d - dr) <= delta) ? 1.0 : 0.0;
    close_neighbors += (depth_in_range(du) && abs(d - du) <= delta) ? 1.0 : 0.0;
    close_neighbors += (depth_in_range(dd) && abs(d - dd) <= delta) ? 1.0 : 0.0;
    return close_neighbors;
}

float close_neighbor_count(vec2 uv, float d) {
    return close_neighbor_count_delta(uv, d, point_cleanup_depth_delta);
}

bool corner_supported(vec2 uv, float d) {
    float guard_delta = max(0.002, max_depth_delta * abs(d) * 0.5);
    return close_neighbor_count_delta(uv, d, guard_delta) >= 2.0;
}

bool triangle_support_ok(vec2 ua, float da, vec2 ub, float db, vec2 uc, float dc) {
    return corner_supported(ua, da) && corner_supported(ub, db) && corner_supported(uc, dc);
}

bool inside_robot_capsule(vec3 point, vec3 a, vec3 b, float radius) {
    vec3 segment = b - a;
    float length_sq = max(dot(segment, segment), 0.0000001);
    float along = clamp(dot(point - a, segment) / length_sq, 0.0, 1.0);
    vec3 closest = a + segment * along;
    return dot(point - closest, point - closest) <= radius * radius;
}

bool robot_masked(vec3 point) {
    if (!robot_mask_enabled) {
        return false;
    }
    for (int index = 0; index < 6; index++) {
        if (index >= robot_capsule_count) {
            break;
        }
        vec4 a = robot_capsules[index * 2];
        vec4 b = robot_capsules[index * 2 + 1];
        if (inside_robot_capsule(point, a.xyz, b.xyz, max(a.w, b.w))) {
            return true;
        }
    }
    return false;
}

void vertex() {
    point_uv = UV;
    tri_uv_a = UV2;
    tri_uv_b = CUSTOM0.xy;
    tri_uv_c = CUSTOM0.zw;
    float da = texture(depth_tex, tri_uv_a).r;
    float db = texture(depth_tex, tri_uv_b).r;
    float dc = texture(depth_tex, tri_uv_c).r;
    vec3 pa = project_uv(tri_uv_a, da);
    vec3 pb = project_uv(tri_uv_b, db);
    vec3 pc = project_uv(tri_uv_c, dc);
    bool tri_ok = triangle_depth_ok(da, db, dc) && triangle_edge_ok(pa, pb, pc) && triangle_support_ok(tri_uv_a, da, tri_uv_b, db, tri_uv_c, dc);
    point_valid = tri_ok ? 1.0 : 0.0;
    if (!tri_ok) {
        float fallback_depth = depth_in_range(da) ? da : (depth_in_range(db) ? db : (depth_in_range(dc) ? dc : depth_range.y));
        VERTEX = project_uv((tri_uv_a + tri_uv_b + tri_uv_c) / 3.0, fallback_depth);
    } else {
        float depth_m = texture(depth_tex, UV).r;
        VERTEX = project_uv(UV, depth_m);
    }
    point_position_local = VERTEX;
}

void fragment() {
    if (point_valid < 0.5) {
        discard;
    }
    if (robot_masked(point_position_local)) {
        discard;
    }

    float da = texture(depth_tex, tri_uv_a).r;
    float db = texture(depth_tex, tri_uv_b).r;
    float dc = texture(depth_tex, tri_uv_c).r;
    if (!triangle_depth_ok(da, db, dc) || !triangle_support_ok(tri_uv_a, da, tri_uv_b, db, tri_uv_c, dc)) {
        discard;
    }
    if (point_cleanup_enabled) {
        float center_depth = texture(depth_tex, point_uv).r;
        if (!depth_in_range(center_depth) || close_neighbor_count(point_uv, center_depth) < point_cleanup_min_neighbors) {
            discard;
        }
    }

    vec3 pa = project_uv(tri_uv_a, da);
    vec3 pb = project_uv(tri_uv_b, db);
    vec3 pc = project_uv(tri_uv_c, dc);
    if (!triangle_edge_ok(pa, pb, pc)) {
        discard;
    }

    float edge_shade = 1.0;
    if (edge_feather_enabled) {
        float max_jump = max(max(abs(da - db), abs(db - dc)), abs(dc - da));
        float max_delta = triangle_delta_limit(da, db, dc);
        float feather = 1.0 - smoothstep(max(max_delta - edge_feather_width, 0.0), max_delta, max_jump);
        edge_shade = mix(edge_feather_min_alpha, 1.0, feather);
    }

    vec4 color = texture(color_tex, point_uv);
    bool color_from_peer = false;
    bool color_valid = !color_enabled || color.a >= 0.5;
    if (fusion_enabled) {
        vec3 peer_point = (fusion_local_to_peer * vec4(point_position_local, 1.0)).xyz;
        float peer_projected_depth = -peer_point.z;
        if (peer_projected_depth > 0.001 && fusion_peer_grid_size.x > 1.0 && fusion_peer_grid_size.y > 1.0) {
            vec2 peer_pixel = vec2(
                peer_point.x * fusion_peer_intrinsics.x / peer_projected_depth + fusion_peer_intrinsics.z,
                -peer_point.y * fusion_peer_intrinsics.y / peer_projected_depth + fusion_peer_intrinsics.w
            );
            vec2 peer_uv = (peer_pixel / max(fusion_peer_grid_stride, 1.0) + vec2(0.5)) / fusion_peer_grid_size;
            if (all(greaterThan(peer_uv, vec2(0.0))) && all(lessThan(peer_uv, vec2(1.0)))) {
                float peer_depth = texture(fusion_peer_depth_tex, peer_uv).r;
                if (peer_depth >= depth_range.x && peer_depth <= depth_range.y && abs(peer_depth - peer_projected_depth) <= fusion_depth_tolerance) {
                    vec4 peer_color = texture(fusion_peer_color_tex, peer_uv);
                    if (!color_enabled || peer_color.a >= 0.5) {
                        if (fusion_color_only) {
                            // Keep both meshes, but make overlap use one camera's RGB.
                            color = peer_color;
                            color_from_peer = true;
                            color_valid = true;
                        } else {
                            if (color_valid && fusion_priority > 0) {
                                discard;
                            }
                            if (!color_valid) {
                                color = peer_color;
                                color_from_peer = true;
                                color_valid = true;
                            }
                        }
                    }
                }
            }
        }
    }
    if (!color_valid) {
        discard;
    }
    vec3 matched_color = color_from_peer ? color.rgb : matched_rgb(color.rgb);
    float gray = dot(matched_color, vec3(0.299, 0.587, 0.114));
    ALBEDO = (color_enabled ? matched_color : vec3(gray)) * edge_shade;
}
)";
    } else if (want_gpu_mesh) {
        shader_code = R"(
shader_type spatial;
render_mode unshaded, cull_disabled, depth_draw_opaque;

uniform sampler2D depth_tex : filter_nearest;
__COLOR_TEX_UNIFORM__
uniform vec4 intrinsics = vec4(600.0, 600.0, 320.0, 240.0);
uniform vec2 depth_range = vec2(0.2, 4.5);
uniform bool color_enabled = true;
uniform bool color_match_enabled = false;
uniform vec3 color_gain = vec3(1.0);
uniform vec3 color_bias = vec3(0.0);
uniform bool color_source_is_linear = true;
uniform bool robot_mask_enabled = false;
uniform int robot_capsule_count = 0;
uniform vec4 robot_capsules[12];

varying vec2 point_uv;
varying float point_valid;
varying vec3 point_position_local;

vec3 linear_to_srgb(vec3 value) {
    vec3 low = value * 12.92;
    vec3 high = 1.055 * pow(max(value, vec3(0.0)), vec3(1.0 / 2.4)) - 0.055;
    return mix(low, high, step(vec3(0.0031308), value));
}

vec3 srgb_to_linear(vec3 value) {
    vec3 low = value / 12.92;
    vec3 high = pow(max((value + 0.055) / 1.055, vec3(0.0)), vec3(2.4));
    return mix(low, high, step(vec3(0.04045), value));
}

vec3 matched_rgb(vec3 value) {
    if (!color_match_enabled) return value;
    vec3 working = color_source_is_linear ? linear_to_srgb(value) : value;
    working = clamp(working * color_gain + color_bias, vec3(0.0), vec3(1.0));
    return color_source_is_linear ? srgb_to_linear(working) : working;
}

bool inside_robot_capsule(vec3 point, vec3 a, vec3 b, float radius) {
    vec3 segment = b - a;
    float along = clamp(dot(point - a, segment) / max(dot(segment, segment), 0.0000001), 0.0, 1.0);
    vec3 closest = a + segment * along;
    return dot(point - closest, point - closest) <= radius * radius;
}

bool robot_masked(vec3 point) {
    if (!robot_mask_enabled) return false;
    for (int index = 0; index < 6; index++) {
        if (index >= robot_capsule_count) break;
        vec4 a = robot_capsules[index * 2];
        vec4 b = robot_capsules[index * 2 + 1];
        if (inside_robot_capsule(point, a.xyz, b.xyz, max(a.w, b.w))) return true;
    }
    return false;
}

void vertex() {
    point_uv = UV;
    float depth_m = texture(depth_tex, UV).r;
    point_valid = step(depth_range.x, depth_m) * step(depth_m, depth_range.y);
    vec2 pixel = VERTEX.xy;
    VERTEX = vec3(
        (pixel.x - intrinsics.z) * depth_m / max(0.000001, intrinsics.x),
        -(pixel.y - intrinsics.w) * depth_m / max(0.000001, intrinsics.y),
        -depth_m
    );
    point_position_local = VERTEX;
}

void fragment() {
    if (point_valid < 0.5) {
        discard;
    }
    if (robot_masked(point_position_local)) {
        discard;
    }
    float d = texture(depth_tex, point_uv).r;
    if (d < depth_range.x || d > depth_range.y) {
        discard;
    }
    vec4 color = texture(color_tex, point_uv);
    if (color_enabled && color.a < 0.5) {
        discard;
    }
    vec3 matched_color = matched_rgb(color.rgb);
    float gray = dot(matched_color, vec3(0.299, 0.587, 0.114));
    ALBEDO = color_enabled ? matched_color : vec3(gray);
}
)";
    } else {
        shader_code = R"(
shader_type spatial;
render_mode unshaded, cull_disabled, depth_draw_opaque;

uniform sampler2D depth_tex : filter_nearest;
__COLOR_TEX_UNIFORM__
uniform vec4 intrinsics = vec4(600.0, 600.0, 320.0, 240.0);
uniform vec2 depth_range = vec2(0.2, 4.5);
uniform float point_pixel_size = 2.0;
uniform bool circular_point_splats = false;
uniform bool point_cleanup_enabled = false;
uniform float point_cleanup_depth_delta = 0.06;
uniform float point_cleanup_min_neighbors = 2.0;
uniform vec2 texel_size = vec2(0.01, 0.01);
uniform float max_depth_delta = 0.08;
uniform bool gpu_connected_mesh = false;
uniform bool edge_feather_enabled = false;
uniform float edge_feather_width = 0.035;
uniform float edge_feather_min_alpha = 0.20;
uniform bool color_enabled = true;
uniform bool color_match_enabled = false;
uniform vec3 color_gain = vec3(1.0);
uniform vec3 color_bias = vec3(0.0);
uniform bool color_source_is_linear = true;
uniform bool robot_mask_enabled = false;
uniform int robot_capsule_count = 0;
uniform vec4 robot_capsules[12];

varying vec2 point_uv;
varying float point_valid;
varying float point_depth_m;
varying vec3 point_position_local;

vec3 linear_to_srgb(vec3 value) {
    vec3 low = value * 12.92;
    vec3 high = 1.055 * pow(max(value, vec3(0.0)), vec3(1.0 / 2.4)) - 0.055;
    return mix(low, high, step(vec3(0.0031308), value));
}

vec3 srgb_to_linear(vec3 value) {
    vec3 low = value / 12.92;
    vec3 high = pow(max((value + 0.055) / 1.055, vec3(0.0)), vec3(2.4));
    return mix(low, high, step(vec3(0.04045), value));
}

vec3 matched_rgb(vec3 value) {
    if (!color_match_enabled) return value;
    vec3 working = color_source_is_linear ? linear_to_srgb(value) : value;
    working = clamp(working * color_gain + color_bias, vec3(0.0), vec3(1.0));
    return color_source_is_linear ? srgb_to_linear(working) : working;
}

bool inside_robot_capsule(vec3 point, vec3 a, vec3 b, float radius) {
    vec3 segment = b - a;
    float along = clamp(dot(point - a, segment) / max(dot(segment, segment), 0.0000001), 0.0, 1.0);
    vec3 closest = a + segment * along;
    return dot(point - closest, point - closest) <= radius * radius;
}

bool robot_masked(vec3 point) {
    if (!robot_mask_enabled) return false;
    for (int index = 0; index < 6; index++) {
        if (index >= robot_capsule_count) break;
        vec4 a = robot_capsules[index * 2];
        vec4 b = robot_capsules[index * 2 + 1];
        if (inside_robot_capsule(point, a.xyz, b.xyz, max(a.w, b.w))) return true;
    }
    return false;
}

void vertex() {
    point_uv = UV;
    float depth_m = texture(depth_tex, UV).r;
    point_depth_m = depth_m;
    point_valid = step(depth_range.x, depth_m) * step(depth_m, depth_range.y);
    POINT_SIZE = point_pixel_size;
    vec2 pixel = VERTEX.xy;
    VERTEX = vec3(
        (pixel.x - intrinsics.z) * depth_m / max(0.000001, intrinsics.x),
        -(pixel.y - intrinsics.w) * depth_m / max(0.000001, intrinsics.y),
        -depth_m
    );
    point_position_local = VERTEX;
}

void fragment() {
    if (point_valid < 0.5) {
        discard;
    }
    if (robot_masked(point_position_local)) {
        discard;
    }
    if (!gpu_connected_mesh && circular_point_splats && distance(POINT_COORD, vec2(0.5)) > 0.5) {
        discard;
    }
    float d = texture(depth_tex, point_uv).r;
    if (d < depth_range.x || d > depth_range.y) {
        discard;
    }
    float dl = texture(depth_tex, point_uv + vec2(-texel_size.x, 0.0)).r;
    float dr = texture(depth_tex, point_uv + vec2(texel_size.x, 0.0)).r;
    float du = texture(depth_tex, point_uv + vec2(0.0, -texel_size.y)).r;
    float dd = texture(depth_tex, point_uv + vec2(0.0, texel_size.y)).r;
    float close_neighbors = 0.0;
    close_neighbors += (dl >= depth_range.x && dl <= depth_range.y && abs(d - dl) <= point_cleanup_depth_delta) ? 1.0 : 0.0;
    close_neighbors += (dr >= depth_range.x && dr <= depth_range.y && abs(d - dr) <= point_cleanup_depth_delta) ? 1.0 : 0.0;
    close_neighbors += (du >= depth_range.x && du <= depth_range.y && abs(d - du) <= point_cleanup_depth_delta) ? 1.0 : 0.0;
    close_neighbors += (dd >= depth_range.x && dd <= depth_range.y && abs(d - dd) <= point_cleanup_depth_delta) ? 1.0 : 0.0;
    if (point_cleanup_enabled && close_neighbors < point_cleanup_min_neighbors) {
        discard;
    }
    float edge_shade = 1.0;
    if (gpu_connected_mesh) {
        float max_jump = max(max(abs(d - dl), abs(d - dr)), max(abs(d - du), abs(d - dd)));
        if (d < depth_range.x || d > depth_range.y || (max_jump > max_depth_delta && close_neighbors < 2.0)) {
            discard;
        }
        if (edge_feather_enabled) {
            float feather = 1.0 - smoothstep(max_depth_delta - edge_feather_width, max_depth_delta, max_jump);
            edge_shade = mix(edge_feather_min_alpha, 1.0, feather);
        }
    }
    vec4 color = texture(color_tex, point_uv);
    if (color_enabled && color.a < 0.5) {
        discard;
    }
    vec3 matched_color = matched_rgb(color.rgb);
    float gray = dot(matched_color, vec3(0.299, 0.587, 0.114));
    ALBEDO = (color_enabled ? matched_color : vec3(gray)) * edge_shade;
}
)";
    }
    shader_code = shader_code.replace("__COLOR_TEX_UNIFORM__", color_sampler);
    shader->set_code(shader_code);
    material.instantiate();
    material_gpu_mesh_mode = want_gpu_mesh;
    material_texture_map_mode = texture_map_mesh;
    material_static_shader_mesh_mode = want_static_shader_mesh;
    material->set_shader(shader);
    set_material_override(material);
}

void RealSenseSharedMemoryPointCloud::ensure_vertex_color_material() {
    if (vertex_color_material.is_valid()) {
        return;
    }
    vertex_color_material.instantiate();
    vertex_color_material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
    vertex_color_material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
    vertex_color_material->set_cull_mode(BaseMaterial3D::CULL_DISABLED);
}

void RealSenseSharedMemoryPointCloud::ensure_cpu_point_material() {
    if (cpu_point_material.is_valid()) {
        return;
    }
    Ref<Shader> shader;
    shader.instantiate();
    shader->set_code(R"(
shader_type spatial;
render_mode unshaded, cull_disabled, depth_draw_opaque;

uniform float point_pixel_size = 2.0;
uniform bool circular_point_splats = false;
uniform bool color_enabled = true;

varying vec4 point_color;
varying vec2 point_quad_uv;

void vertex() {
    point_color = COLOR;
    point_quad_uv = UV;
}

void fragment() {
    if (circular_point_splats && distance(point_quad_uv, vec2(0.5)) > 0.5) {
        discard;
    }
    float gray = dot(point_color.rgb, vec3(0.299, 0.587, 0.114));
    ALBEDO = color_enabled ? point_color.rgb : vec3(gray);
}
)");
    cpu_point_material.instantiate();
    cpu_point_material->set_shader(shader);
}

void RealSenseSharedMemoryPointCloud::ensure_texture_material() {
    if (texture_material.is_null()) {
        texture_material.instantiate();
        texture_material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
        texture_material->set_cull_mode(BaseMaterial3D::CULL_DISABLED);
    }
    if (color_texture.is_valid()) {
        texture_material->set_texture(BaseMaterial3D::TEXTURE_ALBEDO, color_texture);
    }
}

void RealSenseSharedMemoryPointCloud::rebuild_mesh(int p_width, int p_height, int p_stride) {
    grid_width = p_width;
    grid_height = p_height;
    grid_stride = p_stride;

    PackedVector3Array vertices;
    PackedVector2Array uvs;
    PackedVector2Array uv2s;
    PackedColorArray colors;
    PackedFloat32Array custom0s;
    PackedInt32Array indices;
    if (render_connected_mesh && gpu_connected_mesh) {
        const int cell_count = (p_width - 1) * (p_height - 1);
        const int vertex_count = cell_count * 6;
        vertices.resize(vertex_count);
        uvs.resize(vertex_count);
        uv2s.resize(vertex_count);
        custom0s.resize(vertex_count * 4);

        auto pixel_for = [p_width, p_stride](int p_index) -> Vector3 {
            const int y = p_index / p_width;
            const int x = p_index - y * p_width;
            return Vector3(float(x * p_stride), float(y * p_stride), 0.0f);
        };
        auto uv_for = [p_width, p_height](int p_index) -> Vector2 {
            const int y = p_index / p_width;
            const int x = p_index - y * p_width;
            return Vector2((float(x) + 0.5f) / float(p_width), (float(y) + 0.5f) / float(p_height));
        };
        int write_index = 0;
        auto add_tri = [&](int p_a, int p_b, int p_c) {
            const Vector2 uv_a = uv_for(p_a);
            const Vector2 uv_b = uv_for(p_b);
            const Vector2 uv_c = uv_for(p_c);
            const int tri_indices[3] = {p_a, p_b, p_c};
            for (int i = 0; i < 3; ++i) {
                const int src_index = tri_indices[i];
                vertices.set(write_index, pixel_for(src_index));
                uvs.set(write_index, uv_for(src_index));
                uv2s.set(write_index, uv_a);
                custom0s.set(write_index * 4 + 0, uv_b.x);
                custom0s.set(write_index * 4 + 1, uv_b.y);
                custom0s.set(write_index * 4 + 2, uv_c.x);
                custom0s.set(write_index * 4 + 3, uv_c.y);
                write_index++;
            }
        };
        for (int y = 0; y < p_height - 1; y++) {
            for (int x = 0; x < p_width - 1; x++) {
                const int a = y * p_width + x;
                const int b = a + 1;
                const int c = a + p_width;
                const int d = c + 1;
                add_tri(a, c, b);
                add_tri(b, c, d);
            }
        }

        Array arrays;
        arrays.resize(Mesh::ARRAY_MAX);
        arrays[Mesh::ARRAY_VERTEX] = vertices;
        arrays[Mesh::ARRAY_TEX_UV] = uvs;
        arrays[Mesh::ARRAY_TEX_UV2] = uv2s;
        arrays[Mesh::ARRAY_CUSTOM0] = custom0s;
        Ref<ArrayMesh> mesh;
        mesh.instantiate();
        const int64_t custom_flags = int64_t(Mesh::ARRAY_FORMAT_CUSTOM0) | (int64_t(Mesh::ARRAY_CUSTOM_RGBA_FLOAT) << Mesh::ARRAY_FORMAT_CUSTOM0_SHIFT);
        mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays, TypedArray<Array>(), Dictionary(), BitField<Mesh::ArrayFormat>(custom_flags));
        ensure_material();
        mesh->surface_set_material(0, material);
        set_mesh(mesh);
        set_material_override(Ref<Material>());
        set_custom_aabb(AABB(Vector3(-100.0, -100.0, -100.0), Vector3(200.0, 200.0, 200.0)));
        return;
    }

    int count = p_width * p_height;
    vertices.resize(count);
    uvs.resize(count);
    int index = 0;
    for (int y = 0; y < p_height; y++) {
        for (int x = 0; x < p_width; x++) {
            vertices.set(index, Vector3(float(x * p_stride), float(y * p_stride), 0.0f));
            uvs.set(index, Vector2((float(x) + 0.5f) / float(p_width), (float(y) + 0.5f) / float(p_height)));
            index++;
        }
    }
    if (render_connected_mesh && gpu_connected_mesh) {
        indices.resize((p_width - 1) * (p_height - 1) * 6);
        int write_index = 0;
        for (int y = 0; y < p_height - 1; y++) {
            for (int x = 0; x < p_width - 1; x++) {
                const int a = y * p_width + x;
                const int b = a + 1;
                const int c = a + p_width;
                const int d = c + 1;
                indices.set(write_index++, a);
                indices.set(write_index++, c);
                indices.set(write_index++, b);
                indices.set(write_index++, b);
                indices.set(write_index++, c);
                indices.set(write_index++, d);
            }
        }
    }

    Array arrays;
    arrays.resize(Mesh::ARRAY_MAX);
    arrays[Mesh::ARRAY_VERTEX] = vertices;
    arrays[Mesh::ARRAY_TEX_UV] = uvs;
    if (render_connected_mesh && gpu_connected_mesh) {
        arrays[Mesh::ARRAY_INDEX] = indices;
    }
    Ref<ArrayMesh> mesh;
    mesh.instantiate();
    mesh->add_surface_from_arrays((render_connected_mesh && gpu_connected_mesh) ? Mesh::PRIMITIVE_TRIANGLES : Mesh::PRIMITIVE_POINTS, arrays);
    ensure_material();
    mesh->surface_set_material(0, material);
    set_mesh(mesh);
    set_material_override(Ref<Material>());
    set_custom_aabb(AABB(Vector3(-100.0, -100.0, -100.0), Vector3(200.0, 200.0, 200.0)));
}

double RealSenseSharedMemoryPointCloud::now_seconds() const {
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

RealSenseSharedMemoryPointCloud::FrameSnapshot RealSenseSharedMemoryPointCloud::make_reader_snapshot(
    const Ref<RealSenseSharedMemoryReader> &p_reader,
    const Ref<Image> &p_depth_image,
    const Ref<Image> &p_color_image
) const {
    FrameSnapshot frame;
    if (p_reader.is_valid()) {
        frame.sequence = p_reader->get_sequence();
        frame.width = p_reader->get_width();
        frame.height = p_reader->get_height();
        frame.stride = p_reader->get_stride();
        frame.intrinsics = p_reader->get_intrinsics();
    }
    frame.depth_image = p_depth_image;
    frame.color_image = p_color_image;
    frame.timestamp_sec = now_seconds();
    return frame;
}

RealSenseSharedMemoryPointCloud::FrameSnapshot RealSenseSharedMemoryPointCloud::make_direct_snapshot(
    const Ref<RealSenseDirectFrameSource> &p_source,
    const Ref<Image> &p_depth_image,
    const Ref<Image> &p_color_image
) const {
    FrameSnapshot frame;
    if (p_source.is_valid()) {
        frame.sequence = p_source->get_sequence();
        frame.width = p_source->get_width();
        frame.height = p_source->get_height();
        frame.stride = p_source->get_stride();
        frame.intrinsics = p_source->get_intrinsics();
    }
    frame.depth_image = p_depth_image;
    frame.color_image = p_color_image;
    frame.timestamp_sec = now_seconds();
    return frame;
}

void RealSenseSharedMemoryPointCloud::push_frame_history(
    std::deque<FrameSnapshot> &p_history,
    const FrameSnapshot &p_frame
) {
    if (!delay_enabled || p_frame.depth_image.is_null()) {
        return;
    }

    if (!p_history.empty() && p_history.back().sequence == p_frame.sequence) {
        p_history.back() = p_frame;
    } else {
        p_history.push_back(p_frame);
    }

    const double max_delay_sec = std::max(primary_delay_ms, secondary_delay_ms) * 0.001 + 2.0;
    while (p_history.size() > 2 && p_frame.timestamp_sec - p_history.front().timestamp_sec > max_delay_sec) {
        p_history.pop_front();
    }
}

void RealSenseSharedMemoryPointCloud::push_reader_frame_history(
    std::deque<FrameSnapshot> &p_history,
    const Ref<RealSenseSharedMemoryReader> &p_reader,
    const Ref<Image> &p_depth_image,
    const Ref<Image> &p_color_image
) {
    push_frame_history(p_history, make_reader_snapshot(p_reader, p_depth_image, p_color_image));
}

RealSenseSharedMemoryPointCloud::FrameSnapshot RealSenseSharedMemoryPointCloud::select_frame_for_delay(
    const std::deque<FrameSnapshot> &p_history,
    double p_delay_ms,
    const FrameSnapshot &p_latest
) const {
    if (!delay_enabled || p_delay_ms <= 0.0 || p_history.empty()) {
        return p_latest;
    }

    const double target_sec = now_seconds() - (p_delay_ms * 0.001);
    const FrameSnapshot *selected = &p_history.front();
    for (const FrameSnapshot &frame : p_history) {
        if (frame.timestamp_sec <= target_sec) {
            selected = &frame;
        } else {
            break;
        }
    }
    return *selected;
}

int RealSenseSharedMemoryPointCloud::rebuild_cpu_connected_mesh(const Ref<Image> &p_depth_image, const Ref<Image> &p_color_image) {
    int width = current_width;
    int height = current_height;
    int stride = current_stride;
    if (width <= 1 || height <= 1) {
        set_mesh(Ref<Mesh>());
        return 0;
    }

    PackedByteArray depth_data = p_depth_image->get_data();
    PackedByteArray color_data = p_color_image->get_data();
    const int cell_count = width * height;
    if (depth_data.size() < cell_count * 4 || color_data.size() < cell_count * 4) {
        set_mesh(Ref<Mesh>());
        return 0;
    }

    Vector4 intrinsics = current_intrinsics;
    const double fx = MAX(0.000001, double(intrinsics.x));
    const double fy = MAX(0.000001, double(intrinsics.y));
    const double ppx = intrinsics.z;
    const double ppy = intrinsics.w;

    PackedInt32Array compact_index;
    compact_index.resize(cell_count);
    for (int i = 0; i < cell_count; ++i) {
        compact_index.set(i, -1);
    }

    PackedVector3Array vertices;
    PackedColorArray colors;
    PackedVector2Array uvs;
    vertices.resize(cell_count);
    colors.resize(cell_count);
    uvs.resize(cell_count);
    int kept_count = 0;
    for (int y = 0; y < height; ++y) {
        const double py = double(y * stride);
        for (int x = 0; x < width; ++x) {
            const int source_index = y * width + x;
            const float depth_m = depth_data.decode_float(source_index * 4);
            if (depth_m < min_depth || depth_m > max_depth) {
                continue;
            }
            const double px = double(x * stride);
            compact_index.set(source_index, kept_count);
            vertices.set(kept_count, Vector3(
                float((px - ppx) * depth_m / fx),
                float(-(py - ppy) * depth_m / fy),
                -depth_m
            ));
            uvs.set(kept_count, Vector2((float(x) + 0.5f) / float(width), (float(y) + 0.5f) / float(height)));
            const int color_offset = source_index * 4;
            colors.set(kept_count, decode_point_color(color_data, color_offset));
            kept_count++;
        }
    }
    vertices.resize(kept_count);
    colors.resize(kept_count);
    uvs.resize(kept_count);
    if (kept_count <= 2) {
        set_mesh(Ref<Mesh>());
        last_point_count = kept_count;
        return 0;
    }

    PackedInt32Array indices;
    indices.resize((width - 1) * (height - 1) * 6);
    const double max_edge_sq = mesh_max_edge * mesh_max_edge;
    int write_index = 0;
    for (int y = 0; y < height - 1; ++y) {
        for (int x = 0; x < width - 1; ++x) {
            const int a = compact_index[y * width + x];
            const int b = compact_index[y * width + x + 1];
            const int c = compact_index[(y + 1) * width + x];
            const int d = compact_index[(y + 1) * width + x + 1];
            const float da = depth_data.decode_float((y * width + x) * 4);
            const float db = depth_data.decode_float((y * width + x + 1) * 4);
            const float dc = depth_data.decode_float(((y + 1) * width + x) * 4);
            const float dd = depth_data.decode_float(((y + 1) * width + x + 1) * 4);
            const bool first_depth_ok = Math::abs(double(da - dc)) <= mesh_max_depth_delta && Math::abs(double(dc - db)) <= mesh_max_depth_delta && Math::abs(double(db - da)) <= mesh_max_depth_delta;
            const bool second_depth_ok = Math::abs(double(db - dc)) <= mesh_max_depth_delta && Math::abs(double(dc - dd)) <= mesh_max_depth_delta && Math::abs(double(dd - db)) <= mesh_max_depth_delta;
            if (a >= 0 && b >= 0 && c >= 0 && first_depth_ok && triangle_valid(vertices, a, c, b, max_edge_sq) && triangle_color_valid(colors, a, c, b)) {
                indices.set(write_index++, a);
                indices.set(write_index++, c);
                indices.set(write_index++, b);
            }
            if (b >= 0 && c >= 0 && d >= 0 && second_depth_ok && triangle_valid(vertices, b, c, d, max_edge_sq) && triangle_color_valid(colors, b, c, d)) {
                indices.set(write_index++, b);
                indices.set(write_index++, c);
                indices.set(write_index++, d);
            }
        }
    }
    indices.resize(write_index);

    Array arrays;
    arrays.resize(Mesh::ARRAY_MAX);
    arrays[Mesh::ARRAY_VERTEX] = vertices;
    if (texture_map_mesh && color_enabled) {
        arrays[Mesh::ARRAY_TEX_UV] = uvs;
    } else {
        arrays[Mesh::ARRAY_COLOR] = colors;
    }
    arrays[Mesh::ARRAY_INDEX] = indices;
    Ref<ArrayMesh> mesh;
    mesh.instantiate();
    mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
    if (texture_map_mesh && color_enabled) {
        if (color_texture.is_null()) {
            color_texture = ImageTexture::create_from_image(p_color_image);
        } else {
            color_texture->update(p_color_image);
        }
        ensure_texture_material();
        mesh->surface_set_material(0, texture_material);
    } else {
        ensure_vertex_color_material();
        mesh->surface_set_material(0, vertex_color_material);
    }
    set_mesh(mesh);
    set_material_override(Ref<Material>());
    set_custom_aabb(AABB(Vector3(-100.0, -100.0, -100.0), Vector3(200.0, 200.0, 200.0)));
    last_point_count = kept_count;
    return write_index / 3;
}

void RealSenseSharedMemoryPointCloud::release_gpu_mesh_compute_resources() {
    if (mesh_compute_rd == nullptr) {
        mesh_compute_shader = RID();
        mesh_compute_pipeline = RID();
        mesh_compute_depth_buffer = RID();
        mesh_compute_index_buffer = RID();
        mesh_compute_counter_buffer = RID();
        mesh_compute_uniform_set = RID();
        mesh_compute_width = 0;
        mesh_compute_height = 0;
        return;
    }
    if (mesh_compute_uniform_set.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_uniform_set);
        mesh_compute_uniform_set = RID();
    }
    if (mesh_compute_counter_buffer.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_counter_buffer);
        mesh_compute_counter_buffer = RID();
    }
    if (mesh_compute_index_buffer.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_index_buffer);
        mesh_compute_index_buffer = RID();
    }
    if (mesh_compute_depth_buffer.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_depth_buffer);
        mesh_compute_depth_buffer = RID();
    }
    if (mesh_compute_pipeline.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_pipeline);
        mesh_compute_pipeline = RID();
    }
    if (mesh_compute_shader.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_shader);
        mesh_compute_shader = RID();
    }
    mesh_compute_width = 0;
    mesh_compute_height = 0;
}

bool RealSenseSharedMemoryPointCloud::ensure_gpu_mesh_compute_resources(int p_width, int p_height) {
    if (mesh_compute_failed || p_width <= 1 || p_height <= 1) {
        return false;
    }
    if (mesh_compute_rd == nullptr) {
        RenderingServer *server = RenderingServer::get_singleton();
        if (server == nullptr) {
            mesh_compute_failed = true;
            return false;
        }
        mesh_compute_rd = server->create_local_rendering_device();
        if (mesh_compute_rd == nullptr) {
            UtilityFunctions::push_warning("GPU mesh compute indices unavailable: could not create local RenderingDevice.");
            mesh_compute_failed = true;
            return false;
        }
    }

    if (!mesh_compute_shader.is_valid()) {
        const String shader_source = R"GLSL(
#version 450

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

layout(set = 0, binding = 0, std430) readonly buffer DepthBuffer {
    float value[];
} depth_buffer;

layout(set = 0, binding = 1, std430) buffer IndexBuffer {
    uint value[];
} index_buffer;

layout(set = 0, binding = 2, std430) buffer CounterBuffer {
    uint value;
} counter_buffer;

layout(push_constant, std430) uniform Params {
    int width;
    int height;
    int stride;
    float min_depth;
    float max_depth;
    float max_depth_delta;
    float max_edge_sq;
    float fx;
    float fy;
    float ppx;
    float ppy;
    uint padding0;
} params;

bool valid_depth(float depth) {
    return !isnan(depth) && !isinf(depth) && depth >= params.min_depth && depth <= params.max_depth;
}

vec3 projected(int x, int y, float depth) {
    float px = float(x * params.stride);
    float py = float(y * params.stride);
    return vec3(
        (px - params.ppx) * depth / params.fx,
        -(py - params.ppy) * depth / params.fy,
        -depth
    );
}

bool depth_delta_ok(float a, float b, float c) {
    return abs(a - b) <= params.max_depth_delta
        && abs(b - c) <= params.max_depth_delta
        && abs(c - a) <= params.max_depth_delta;
}

bool edge_ok(vec3 a, vec3 b, vec3 c) {
    vec3 ab = a - b;
    vec3 bc = b - c;
    vec3 ca = c - a;
    return dot(ab, ab) <= params.max_edge_sq
        && dot(bc, bc) <= params.max_edge_sq
        && dot(ca, ca) <= params.max_edge_sq;
}

void emit_triangle(uint a, uint b, uint c) {
    uint dst = atomicAdd(counter_buffer.value, 3u);
    index_buffer.value[dst] = a;
    index_buffer.value[dst + 1u] = b;
    index_buffer.value[dst + 2u] = c;
}

void main() {
    int x = int(gl_GlobalInvocationID.x);
    int y = int(gl_GlobalInvocationID.y);
    if (x >= params.width - 1 || y >= params.height - 1) {
        return;
    }

    int ia = y * params.width + x;
    int ib = ia + 1;
    int ic = ia + params.width;
    int id = ic + 1;
    float da = depth_buffer.value[ia];
    float db = depth_buffer.value[ib];
    float dc = depth_buffer.value[ic];
    float dd = depth_buffer.value[id];

    if (valid_depth(da) && valid_depth(dc) && valid_depth(db) && depth_delta_ok(da, dc, db)) {
        vec3 pa = projected(x, y, da);
        vec3 pc = projected(x, y + 1, dc);
        vec3 pb = projected(x + 1, y, db);
        if (edge_ok(pa, pc, pb)) {
            emit_triangle(uint(ia), uint(ic), uint(ib));
        }
    }
    if (valid_depth(db) && valid_depth(dc) && valid_depth(dd) && depth_delta_ok(db, dc, dd)) {
        vec3 pb = projected(x + 1, y, db);
        vec3 pc = projected(x, y + 1, dc);
        vec3 pd = projected(x + 1, y + 1, dd);
        if (edge_ok(pb, pc, pd)) {
            emit_triangle(uint(ib), uint(ic), uint(id));
        }
    }
}
)GLSL";
        Ref<RDShaderSource> source;
        source.instantiate();
        source->set_language(RenderingDevice::SHADER_LANGUAGE_GLSL);
        source->set_stage_source(RenderingDevice::SHADER_STAGE_COMPUTE, shader_source);
        Ref<RDShaderSPIRV> spirv = mesh_compute_rd->shader_compile_spirv_from_source(source);
        if (spirv.is_null()) {
            UtilityFunctions::push_warning("GPU mesh compute indices unavailable: compute shader SPIR-V compile returned null.");
            mesh_compute_failed = true;
            return false;
        }
        const String compile_error = spirv->get_stage_compile_error(RenderingDevice::SHADER_STAGE_COMPUTE);
        if (!compile_error.is_empty()) {
            UtilityFunctions::push_warning(String("GPU mesh compute indices unavailable: ") + compile_error);
            mesh_compute_failed = true;
            return false;
        }
        mesh_compute_shader = mesh_compute_rd->shader_create_from_spirv(spirv, "point_cloud_mesh_indices");
        if (!mesh_compute_shader.is_valid()) {
            UtilityFunctions::push_warning("GPU mesh compute indices unavailable: could not create compute shader.");
            mesh_compute_failed = true;
            return false;
        }
        mesh_compute_pipeline = mesh_compute_rd->compute_pipeline_create(mesh_compute_shader);
        if (!mesh_compute_pipeline.is_valid() || !mesh_compute_rd->compute_pipeline_is_valid(mesh_compute_pipeline)) {
            UtilityFunctions::push_warning("GPU mesh compute indices unavailable: could not create compute pipeline.");
            mesh_compute_failed = true;
            return false;
        }
    }

    if (
        mesh_compute_width == p_width
        && mesh_compute_height == p_height
        && mesh_compute_depth_buffer.is_valid()
        && mesh_compute_index_buffer.is_valid()
        && mesh_compute_counter_buffer.is_valid()
        && mesh_compute_uniform_set.is_valid()
        && mesh_compute_rd->uniform_set_is_valid(mesh_compute_uniform_set)
    ) {
        return true;
    }

    if (mesh_compute_uniform_set.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_uniform_set);
        mesh_compute_uniform_set = RID();
    }
    if (mesh_compute_counter_buffer.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_counter_buffer);
        mesh_compute_counter_buffer = RID();
    }
    if (mesh_compute_index_buffer.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_index_buffer);
        mesh_compute_index_buffer = RID();
    }
    if (mesh_compute_depth_buffer.is_valid()) {
        mesh_compute_rd->free_rid(mesh_compute_depth_buffer);
        mesh_compute_depth_buffer = RID();
    }

    const int depth_bytes = p_width * p_height * 4;
    const int max_index_count = (p_width - 1) * (p_height - 1) * 6;
    const int index_bytes = max_index_count * 4;
    PackedByteArray empty_depth;
    empty_depth.resize(depth_bytes);
    PackedByteArray empty_indices;
    empty_indices.resize(index_bytes);
    PackedByteArray counter_zero;
    counter_zero.resize(4);
    counter_zero.encode_u32(0, 0);
    mesh_compute_depth_buffer = mesh_compute_rd->storage_buffer_create(depth_bytes, empty_depth);
    mesh_compute_index_buffer = mesh_compute_rd->storage_buffer_create(index_bytes, empty_indices);
    mesh_compute_counter_buffer = mesh_compute_rd->storage_buffer_create(4, counter_zero);
    if (!mesh_compute_depth_buffer.is_valid() || !mesh_compute_index_buffer.is_valid() || !mesh_compute_counter_buffer.is_valid()) {
        UtilityFunctions::push_warning("GPU mesh compute indices unavailable: could not create storage buffers.");
        mesh_compute_failed = true;
        return false;
    }

    Ref<RDUniform> depth_uniform;
    depth_uniform.instantiate();
    depth_uniform->set_uniform_type(RenderingDevice::UNIFORM_TYPE_STORAGE_BUFFER);
    depth_uniform->set_binding(0);
    depth_uniform->add_id(mesh_compute_depth_buffer);
    Ref<RDUniform> index_uniform;
    index_uniform.instantiate();
    index_uniform->set_uniform_type(RenderingDevice::UNIFORM_TYPE_STORAGE_BUFFER);
    index_uniform->set_binding(1);
    index_uniform->add_id(mesh_compute_index_buffer);
    Ref<RDUniform> counter_uniform;
    counter_uniform.instantiate();
    counter_uniform->set_uniform_type(RenderingDevice::UNIFORM_TYPE_STORAGE_BUFFER);
    counter_uniform->set_binding(2);
    counter_uniform->add_id(mesh_compute_counter_buffer);
    TypedArray<Ref<RDUniform>> uniforms;
    uniforms.push_back(depth_uniform);
    uniforms.push_back(index_uniform);
    uniforms.push_back(counter_uniform);
    mesh_compute_uniform_set = mesh_compute_rd->uniform_set_create(uniforms, mesh_compute_shader, 0);
    if (!mesh_compute_uniform_set.is_valid() || !mesh_compute_rd->uniform_set_is_valid(mesh_compute_uniform_set)) {
        UtilityFunctions::push_warning("GPU mesh compute indices unavailable: could not create uniform set.");
        mesh_compute_failed = true;
        return false;
    }

    mesh_compute_width = p_width;
    mesh_compute_height = p_height;
    return true;
}

bool RealSenseSharedMemoryPointCloud::rebuild_gpu_mesh_indices_compute(
    const PackedByteArray &p_depth_data,
    int p_width,
    int p_height,
    int p_stride,
    const Vector4 &p_intrinsics,
    PackedInt32Array &r_indices
) {
    r_indices.clear();
    if (!ensure_gpu_mesh_compute_resources(p_width, p_height)) {
        return false;
    }
    const int cell_count = p_width * p_height;
    const int depth_bytes = cell_count * 4;
    const int max_index_count = (p_width - 1) * (p_height - 1) * 6;
    if (p_depth_data.size() < depth_bytes || max_index_count <= 0) {
        return false;
    }

    PackedByteArray counter_zero;
    counter_zero.resize(4);
    counter_zero.encode_u32(0, 0);
    if (mesh_compute_rd->buffer_update(mesh_compute_depth_buffer, 0, depth_bytes, p_depth_data) != OK) {
        return false;
    }
    if (mesh_compute_rd->buffer_update(mesh_compute_counter_buffer, 0, 4, counter_zero) != OK) {
        return false;
    }

    PackedByteArray push_constants;
    push_constants.resize(48);
    push_constants.encode_u32(0, p_width);
    push_constants.encode_u32(4, p_height);
    push_constants.encode_u32(8, p_stride);
    push_constants.encode_float(12, min_depth);
    push_constants.encode_float(16, max_depth);
    push_constants.encode_float(20, mesh_max_depth_delta);
    push_constants.encode_float(24, mesh_max_edge * mesh_max_edge);
    push_constants.encode_float(28, MAX(0.000001, double(p_intrinsics.x)));
    push_constants.encode_float(32, MAX(0.000001, double(p_intrinsics.y)));
    push_constants.encode_float(36, p_intrinsics.z);
    push_constants.encode_float(40, p_intrinsics.w);
    push_constants.encode_u32(44, 0);

    const int64_t compute_list = mesh_compute_rd->compute_list_begin();
    mesh_compute_rd->compute_list_bind_compute_pipeline(compute_list, mesh_compute_pipeline);
    mesh_compute_rd->compute_list_bind_uniform_set(compute_list, mesh_compute_uniform_set, 0);
    mesh_compute_rd->compute_list_set_push_constant(compute_list, push_constants, uint32_t(push_constants.size()));
    mesh_compute_rd->compute_list_dispatch(compute_list, uint32_t((p_width + 15) / 16), uint32_t((p_height + 15) / 16), 1);
    mesh_compute_rd->compute_list_end();
    mesh_compute_rd->submit();
    mesh_compute_rd->sync();

    const PackedByteArray counter_data = mesh_compute_rd->buffer_get_data(mesh_compute_counter_buffer, 0, 4);
    if (counter_data.size() < 4) {
        return false;
    }
    int index_count = int(counter_data.decode_u32(0));
    index_count = std::max(0, std::min(index_count, max_index_count));
    if (index_count == 0) {
        return false;
    }
    const PackedByteArray index_data = mesh_compute_rd->buffer_get_data(mesh_compute_index_buffer, 0, index_count * 4);
    if (index_data.size() < index_count * 4) {
        return false;
    }
    r_indices.resize(index_count);
    for (int i = 0; i < index_count; ++i) {
        r_indices.set(i, int(index_data.decode_u32(i * 4)));
    }
    return true;
}

int RealSenseSharedMemoryPointCloud::rebuild_gpu_connected_mesh(const Ref<Image> &p_depth_image) {
    if (p_depth_image.is_null()) {
        set_mesh(Ref<Mesh>());
        return 0;
    }
    const int width = current_width;
    const int height = current_height;
    const int stride = current_stride;
    if (width <= 1 || height <= 1) {
        set_mesh(Ref<Mesh>());
        return 0;
    }

    PackedByteArray depth_data = p_depth_image->get_data();
    const int cell_count = width * height;
    if (depth_data.size() < cell_count * 4) {
        set_mesh(Ref<Mesh>());
        return 0;
    }

    Vector4 intrinsics = current_intrinsics;
    const double fx = MAX(0.000001, double(intrinsics.x));
    const double fy = MAX(0.000001, double(intrinsics.y));
    const double ppx = intrinsics.z;
    const double ppy = intrinsics.w;
    const double max_edge_sq = mesh_max_edge * mesh_max_edge;

    if (
        gpu_mesh_cache_width != width
        || gpu_mesh_cache_height != height
        || gpu_mesh_cache_stride != stride
        || gpu_mesh_vertices.size() != cell_count
        || gpu_mesh_uvs.size() != cell_count
    ) {
        gpu_mesh_vertices.resize(cell_count);
        gpu_mesh_uvs.resize(cell_count);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const int index = y * width + x;
                gpu_mesh_vertices.set(index, Vector3(float(x * stride), float(y * stride), 0.0f));
                gpu_mesh_uvs.set(index, Vector2((float(x) + 0.5f) / float(width), (float(y) + 0.5f) / float(height)));
            }
        }
        gpu_mesh_cache_width = width;
        gpu_mesh_cache_height = height;
        gpu_mesh_cache_stride = stride;
    }

    auto valid_depth = [&](float p_depth) -> bool {
        return std::isfinite(p_depth) && double(p_depth) >= min_depth && double(p_depth) <= max_depth;
    };
    auto projected = [&](int p_x, int p_y, float p_depth) -> Vector3 {
        const double px = double(p_x * stride);
        const double py = double(p_y * stride);
        return Vector3(
            float((px - ppx) * double(p_depth) / fx),
            float(-(py - ppy) * double(p_depth) / fy),
            -p_depth
        );
    };
    auto edge_ok = [max_edge_sq](const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c) -> bool {
        return p_a.distance_squared_to(p_b) <= max_edge_sq
            && p_b.distance_squared_to(p_c) <= max_edge_sq
            && p_c.distance_squared_to(p_a) <= max_edge_sq;
    };

    int write_index = 0;
    PackedInt32Array indices;
    if (gpu_mesh_compute_indices && rebuild_gpu_mesh_indices_compute(depth_data, width, height, stride, intrinsics, indices)) {
        write_index = indices.size();
    } else {
        indices.resize((width - 1) * (height - 1) * 6);
        for (int y = 0; y < height - 1; ++y) {
            for (int x = 0; x < width - 1; ++x) {
                const int ia = y * width + x;
                const int ib = ia + 1;
                const int ic = ia + width;
                const int id = ic + 1;
                const float da = depth_data.decode_float(ia * 4);
                const float db = depth_data.decode_float(ib * 4);
                const float dc = depth_data.decode_float(ic * 4);
                const float dd = depth_data.decode_float(id * 4);
                const bool first_depth_ok = valid_depth(da) && valid_depth(dc) && valid_depth(db)
                    && Math::abs(double(da - dc)) <= mesh_max_depth_delta
                    && Math::abs(double(dc - db)) <= mesh_max_depth_delta
                    && Math::abs(double(db - da)) <= mesh_max_depth_delta;
                if (first_depth_ok) {
                    const Vector3 pa = projected(x, y, da);
                    const Vector3 pc = projected(x, y + 1, dc);
                    const Vector3 pb = projected(x + 1, y, db);
                    if (edge_ok(pa, pc, pb)) {
                        indices.set(write_index++, ia);
                        indices.set(write_index++, ic);
                        indices.set(write_index++, ib);
                    }
                }
                const bool second_depth_ok = valid_depth(db) && valid_depth(dc) && valid_depth(dd)
                    && Math::abs(double(db - dc)) <= mesh_max_depth_delta
                    && Math::abs(double(dc - dd)) <= mesh_max_depth_delta
                    && Math::abs(double(dd - db)) <= mesh_max_depth_delta;
                if (second_depth_ok) {
                    const Vector3 pb = projected(x + 1, y, db);
                    const Vector3 pc = projected(x, y + 1, dc);
                    const Vector3 pd = projected(x + 1, y + 1, dd);
                    if (edge_ok(pb, pc, pd)) {
                        indices.set(write_index++, ib);
                        indices.set(write_index++, ic);
                        indices.set(write_index++, id);
                    }
                }
            }
        }
        indices.resize(write_index);
    }

    Array arrays;
    arrays.resize(Mesh::ARRAY_MAX);
    arrays[Mesh::ARRAY_VERTEX] = gpu_mesh_vertices;
    arrays[Mesh::ARRAY_TEX_UV] = gpu_mesh_uvs;
    arrays[Mesh::ARRAY_INDEX] = indices;
    Ref<ArrayMesh> mesh;
    mesh.instantiate();
    mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
    ensure_material();
    mesh->surface_set_material(0, material);
    set_mesh(mesh);
    set_material_override(Ref<Material>());
    set_custom_aabb(AABB(Vector3(-100.0, -100.0, -100.0), Vector3(200.0, 200.0, 200.0)));
    return write_index / 3;
}

int RealSenseSharedMemoryPointCloud::rebuild_cpu_point_cloud(const Ref<Image> &p_depth_image, const Ref<Image> &p_color_image) {
    if (p_depth_image.is_null() || p_color_image.is_null()) {
        set_mesh(Ref<Mesh>());
        return 0;
    }
    int width = current_width;
    int height = current_height;
    int stride = current_stride;
    if (width <= 1 || height <= 1) {
        set_mesh(Ref<Mesh>());
        return 0;
    }

    PackedByteArray depth_data = p_depth_image->get_data();
    PackedByteArray color_data = p_color_image->get_data();
    const int cell_count = width * height;
    if (depth_data.size() < cell_count * 4 || color_data.size() < cell_count * 4) {
        set_mesh(Ref<Mesh>());
        return 0;
    }

    Vector4 intrinsics = current_intrinsics;
    const double fx = MAX(0.000001, double(intrinsics.x));
    const double fy = MAX(0.000001, double(intrinsics.y));
    const double ppx = intrinsics.z;
    const double ppy = intrinsics.w;

    PackedVector3Array vertices;
    PackedColorArray colors;
    PackedVector2Array uvs;
    PackedInt32Array indices;
    const int max_vertices = cell_count * 4;
    const int max_indices = cell_count * 6;
    vertices.resize(max_vertices);
    colors.resize(max_vertices);
    uvs.resize(max_vertices);
    indices.resize(max_indices);
    int kept_count = 0;
    int vertex_count = 0;
    int index_count = 0;
    for (int y = 0; y < height; ++y) {
        const double py = double(y * stride);
        for (int x = 0; x < width; ++x) {
            const int source_index = y * width + x;
            const float depth_m = depth_data.decode_float(source_index * 4);
            if (depth_m < min_depth || depth_m > max_depth) {
                continue;
            }
            if (point_cleanup_enabled) {
                int close_neighbors = 0;
                const int nx[4] = { x - 1, x + 1, x, x };
                const int ny[4] = { y, y, y - 1, y + 1 };
                for (int i = 0; i < 4; ++i) {
                    if (nx[i] < 0 || nx[i] >= width || ny[i] < 0 || ny[i] >= height) {
                        continue;
                    }
                    const int neighbor_index = ny[i] * width + nx[i];
                    const float neighbor_depth = depth_data.decode_float(neighbor_index * 4);
                    if (neighbor_depth >= min_depth && neighbor_depth <= max_depth && Math::abs(double(depth_m - neighbor_depth)) <= point_cleanup_depth_delta) {
                        close_neighbors++;
                    }
                }
                if (double(close_neighbors) < point_cleanup_min_neighbors) {
                    continue;
                }
            }
            const double px = double(x * stride);
            const Vector3 point(
                float((px - ppx) * depth_m / fx),
                float(-(py - ppy) * depth_m / fy),
                -depth_m
            );
            const int color_offset = source_index * 4;
            const Color color = decode_point_color(color_data, color_offset);
            const int base = vertex_count;
            const float half_size = float(MAX(0.00025, point_pixel_size * 0.0012 * MAX(0.2, double(depth_m))));
            vertices.set(base + 0, point + Vector3(-half_size, -half_size, 0.0f));
            vertices.set(base + 1, point + Vector3(half_size, -half_size, 0.0f));
            vertices.set(base + 2, point + Vector3(-half_size, half_size, 0.0f));
            vertices.set(base + 3, point + Vector3(half_size, half_size, 0.0f));
            colors.set(base + 0, color);
            colors.set(base + 1, color);
            colors.set(base + 2, color);
            colors.set(base + 3, color);
            uvs.set(base + 0, Vector2(0.0f, 0.0f));
            uvs.set(base + 1, Vector2(1.0f, 0.0f));
            uvs.set(base + 2, Vector2(0.0f, 1.0f));
            uvs.set(base + 3, Vector2(1.0f, 1.0f));
            indices.set(index_count + 0, base + 0);
            indices.set(index_count + 1, base + 2);
            indices.set(index_count + 2, base + 1);
            indices.set(index_count + 3, base + 1);
            indices.set(index_count + 4, base + 2);
            indices.set(index_count + 5, base + 3);
            vertex_count += 4;
            index_count += 6;
            kept_count++;
        }
    }
    vertices.resize(vertex_count);
    colors.resize(vertex_count);
    uvs.resize(vertex_count);
    indices.resize(index_count);
    if (kept_count <= 0) {
        set_mesh(Ref<Mesh>());
        return 0;
    }

    Array arrays;
    arrays.resize(Mesh::ARRAY_MAX);
    arrays[Mesh::ARRAY_VERTEX] = vertices;
    arrays[Mesh::ARRAY_COLOR] = colors;
    arrays[Mesh::ARRAY_TEX_UV] = uvs;
    arrays[Mesh::ARRAY_INDEX] = indices;
    Ref<ArrayMesh> mesh;
    mesh.instantiate();
    mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
    ensure_vertex_color_material();
    mesh->surface_set_material(0, vertex_color_material);
    set_mesh(mesh);
    set_material_override(Ref<Material>());
    set_custom_aabb(AABB(Vector3(-100.0, -100.0, -100.0), Vector3(200.0, 200.0, 200.0)));
    return kept_count;
}

int RealSenseSharedMemoryPointCloud::append_cpu_grid_surface(
    Ref<ArrayMesh> &p_mesh,
    const Ref<RealSenseSharedMemoryReader> &p_reader,
    const Ref<Image> &p_depth_image,
    const Ref<Image> &p_color_image,
    const Transform3D &p_transform,
    int &r_point_count
) {
    if (p_reader.is_null() || p_depth_image.is_null() || p_color_image.is_null()) {
        return 0;
    }
    const int width = p_reader->get_width();
    const int height = p_reader->get_height();
    const int stride = p_reader->get_stride();
    if (width <= 1 || height <= 1) {
        return 0;
    }

    PackedByteArray depth_data = p_depth_image->get_data();
    PackedByteArray color_data = p_color_image->get_data();
    const int cell_count = width * height;
    if (depth_data.size() < cell_count * 4 || color_data.size() < cell_count * 4) {
        return 0;
    }

    Vector4 intrinsics = p_reader->get_intrinsics();
    const double fx = MAX(0.000001, double(intrinsics.x));
    const double fy = MAX(0.000001, double(intrinsics.y));
    const double ppx = intrinsics.z;
    const double ppy = intrinsics.w;

    PackedInt32Array compact_index;
    compact_index.resize(cell_count);
    for (int i = 0; i < cell_count; ++i) {
        compact_index.set(i, -1);
    }

    PackedVector3Array vertices;
    PackedColorArray colors;
    vertices.resize(cell_count);
    colors.resize(cell_count);
    int kept_count = 0;
    for (int y = 0; y < height; ++y) {
        const double py = double(y * stride);
        for (int x = 0; x < width; ++x) {
            const int source_index = y * width + x;
            const float depth_m = depth_data.decode_float(source_index * 4);
            if (depth_m < min_depth || depth_m > max_depth) {
                continue;
            }
            const double px = double(x * stride);
            compact_index.set(source_index, kept_count);
            const Vector3 point(
                float((px - ppx) * depth_m / fx),
                float(-(py - ppy) * depth_m / fy),
                -depth_m
            );
            vertices.set(kept_count, p_transform.xform(point));
            const int color_offset = source_index * 4;
            colors.set(kept_count, decode_point_color(color_data, color_offset));
            kept_count++;
        }
    }
    vertices.resize(kept_count);
    colors.resize(kept_count);
    r_point_count += kept_count;
    if (kept_count <= 2) {
        return 0;
    }

    PackedInt32Array indices;
    indices.resize((width - 1) * (height - 1) * 6);
    const double max_edge_sq = mesh_max_edge * mesh_max_edge;
    int write_index = 0;
    for (int y = 0; y < height - 1; ++y) {
        for (int x = 0; x < width - 1; ++x) {
            const int a = compact_index[y * width + x];
            const int b = compact_index[y * width + x + 1];
            const int c = compact_index[(y + 1) * width + x];
            const int d = compact_index[(y + 1) * width + x + 1];
            const float da = depth_data.decode_float((y * width + x) * 4);
            const float db = depth_data.decode_float((y * width + x + 1) * 4);
            const float dc = depth_data.decode_float(((y + 1) * width + x) * 4);
            const float dd = depth_data.decode_float(((y + 1) * width + x + 1) * 4);
            const bool first_depth_ok = Math::abs(double(da - dc)) <= mesh_max_depth_delta && Math::abs(double(dc - db)) <= mesh_max_depth_delta && Math::abs(double(db - da)) <= mesh_max_depth_delta;
            const bool second_depth_ok = Math::abs(double(db - dc)) <= mesh_max_depth_delta && Math::abs(double(dc - dd)) <= mesh_max_depth_delta && Math::abs(double(dd - db)) <= mesh_max_depth_delta;
            if (a >= 0 && b >= 0 && c >= 0 && first_depth_ok && triangle_valid(vertices, a, c, b, max_edge_sq) && triangle_color_valid(colors, a, c, b)) {
                indices.set(write_index++, a);
                indices.set(write_index++, c);
                indices.set(write_index++, b);
            }
            if (b >= 0 && c >= 0 && d >= 0 && second_depth_ok && triangle_valid(vertices, b, c, d, max_edge_sq) && triangle_color_valid(colors, b, c, d)) {
                indices.set(write_index++, b);
                indices.set(write_index++, c);
                indices.set(write_index++, d);
            }
        }
    }
    indices.resize(write_index);
    if (write_index <= 0) {
        return 0;
    }

    Array arrays;
    arrays.resize(Mesh::ARRAY_MAX);
    arrays[Mesh::ARRAY_VERTEX] = vertices;
    arrays[Mesh::ARRAY_COLOR] = colors;
    arrays[Mesh::ARRAY_INDEX] = indices;
    p_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
    ensure_vertex_color_material();
    p_mesh->surface_set_material(p_mesh->get_surface_count() - 1, vertex_color_material);
    return write_index / 3;
}

int RealSenseSharedMemoryPointCloud::rebuild_cpu_combined_mesh(const Ref<Image> &p_depth_image, const Ref<Image> &p_color_image) {
    Ref<ArrayMesh> mesh;
    mesh.instantiate();
    int point_count = 0;
    int triangle_count = 0;
    if (!direct_realsense_enabled) {
        triangle_count += append_cpu_grid_surface(mesh, reader, p_depth_image, p_color_image, Transform3D(), point_count);
    }

    if (secondary_reader.is_valid() && secondary_reader->is_open()) {
        Ref<Image> secondary_depth_image = secondary_reader->get_depth_image();
        Ref<Image> secondary_color_image = secondary_reader->get_color_image();
        FrameSnapshot secondary_latest = make_reader_snapshot(secondary_reader, secondary_depth_image, secondary_color_image);
        FrameSnapshot secondary_frame = select_frame_for_delay(secondary_history, secondary_delay_ms, secondary_latest);
        triangle_count += append_cpu_grid_surface(mesh, secondary_reader, secondary_frame.depth_image, secondary_frame.color_image, secondary_transform, point_count);
    }

    if (mesh->get_surface_count() <= 0) {
        set_mesh(Ref<Mesh>());
        last_point_count = point_count;
        return 0;
    }
    set_mesh(mesh);
    set_material_override(Ref<Material>());
    set_custom_aabb(AABB(Vector3(-100.0, -100.0, -100.0), Vector3(200.0, 200.0, 200.0)));
    last_point_count = point_count;
    return triangle_count;
}

bool RealSenseSharedMemoryPointCloud::triangle_valid(const PackedVector3Array &p_vertices, int p_a, int p_b, int p_c, double p_max_edge_sq) const {
    const Vector3 pa = p_vertices[p_a];
    const Vector3 pb = p_vertices[p_b];
    const Vector3 pc = p_vertices[p_c];
    if (pa.distance_squared_to(pb) > p_max_edge_sq || pb.distance_squared_to(pc) > p_max_edge_sq || pc.distance_squared_to(pa) > p_max_edge_sq) {
        return false;
    }
    if (mesh_min_triangle_area <= 0.0) {
        return true;
    }
    const double area = 0.5 * double((pb - pa).cross(pc - pa).length());
    return area >= mesh_min_triangle_area;
}

bool RealSenseSharedMemoryPointCloud::triangle_color_valid(const PackedColorArray &p_colors, int p_a, int p_b, int p_c) const {
    if (mesh_max_color_delta <= 0.0 || mesh_max_color_delta >= 2.0) {
        return true;
    }
    const Color ca = p_colors[p_a];
    const Color cb = p_colors[p_b];
    const Color cc = p_colors[p_c];
    const Vector3 va(ca.r, ca.g, ca.b);
    const Vector3 vb(cb.r, cb.g, cb.b);
    const Vector3 vc(cc.r, cc.g, cc.b);
    return va.distance_to(vb) <= mesh_max_color_delta && vb.distance_to(vc) <= mesh_max_color_delta && vc.distance_to(va) <= mesh_max_color_delta;
}

Color RealSenseSharedMemoryPointCloud::decode_point_color(const PackedByteArray &p_color_data, int p_color_offset) const {
    float r = float(uint8_t(p_color_data[p_color_offset])) / 255.0f;
    float g = float(uint8_t(p_color_data[p_color_offset + 1])) / 255.0f;
    float b = float(uint8_t(p_color_data[p_color_offset + 2])) / 255.0f;
    const float a = float(uint8_t(p_color_data[p_color_offset + 3])) / 255.0f;
    if (color_match_enabled) {
        r = std::clamp(r * color_gain.x + color_bias.x, 0.0f, 1.0f);
        g = std::clamp(g * color_gain.y + color_bias.y, 0.0f, 1.0f);
        b = std::clamp(b * color_gain.z + color_bias.z, 0.0f, 1.0f);
    }
    if (color_enabled) {
        return Color(r, g, b, a);
    }
    const float gray = r * 0.299f + g * 0.587f + b * 0.114f;
    return Color(gray, gray, gray, a);
}

void RealSenseSharedMemoryPointCloud::update_material_params() {
    if (material.is_null()) {
        return;
    }
    if (depth_texture.is_valid()) {
        material->set_shader_parameter("depth_tex", depth_texture);
    }
    if (color_texture.is_valid()) {
        material->set_shader_parameter("color_tex", color_texture);
    }
    material->set_shader_parameter("intrinsics", current_intrinsics);
    material->set_shader_parameter("depth_range", Vector2(float(min_depth), float(max_depth)));
    material->set_shader_parameter("point_pixel_size", float(point_pixel_size));
    material->set_shader_parameter("circular_point_splats", circular_point_splats);
    material->set_shader_parameter("point_cleanup_enabled", point_cleanup_enabled);
    material->set_shader_parameter("point_cleanup_depth_delta", float(point_cleanup_depth_delta));
    material->set_shader_parameter("point_cleanup_min_neighbors", float(point_cleanup_min_neighbors));
    material->set_shader_parameter("texel_size", Vector2(grid_width > 0 ? 1.0f / float(grid_width) : 1.0f, grid_height > 0 ? 1.0f / float(grid_height) : 1.0f));
    material->set_shader_parameter("grid_stride", float(grid_stride > 0 ? grid_stride : 1));
    material->set_shader_parameter("max_depth_delta", float(mesh_max_depth_delta));
    material->set_shader_parameter("max_mesh_edge", float(mesh_max_edge));
    material->set_shader_parameter("gpu_connected_mesh", gpu_connected_mesh && render_connected_mesh);
    material->set_shader_parameter("edge_feather_enabled", edge_feather_enabled);
    material->set_shader_parameter("edge_feather_width", float(edge_feather_width));
    material->set_shader_parameter("edge_feather_min_alpha", float(edge_feather_min_alpha));
    material->set_shader_parameter("color_enabled", color_enabled);
    material->set_shader_parameter("color_match_enabled", color_match_enabled);
    material->set_shader_parameter("color_gain", color_gain);
    material->set_shader_parameter("color_bias", color_bias);
    material->set_shader_parameter("color_source_is_linear", texture_map_mesh);
    const bool fusion_ready = fusion_enabled && fusion_peer_depth_texture.is_valid() && fusion_peer_grid_size.x > 1.0 && fusion_peer_grid_size.y > 1.0;
    material->set_shader_parameter("fusion_enabled", fusion_ready);
    material->set_shader_parameter("fusion_color_only", fusion_color_only);
    if (fusion_peer_depth_texture.is_valid()) {
        material->set_shader_parameter("fusion_peer_depth_tex", fusion_peer_depth_texture);
    }
    if (fusion_peer_color_texture.is_valid()) {
        material->set_shader_parameter("fusion_peer_color_tex", fusion_peer_color_texture);
    }
    material->set_shader_parameter("fusion_local_to_peer", fusion_local_to_peer);
    material->set_shader_parameter("fusion_peer_intrinsics", fusion_peer_intrinsics);
    material->set_shader_parameter("fusion_peer_grid_size", fusion_peer_grid_size);
    material->set_shader_parameter("fusion_peer_grid_stride", float(fusion_peer_grid_stride));
    material->set_shader_parameter("fusion_depth_tolerance", float(fusion_depth_tolerance));
    material->set_shader_parameter("fusion_priority", fusion_priority);
    PackedVector4Array padded_capsules;
    padded_capsules.resize(12);
    const int capsule_values = std::min(12, int(robot_mask_capsules.size()));
    for (int index = 0; index < capsule_values; ++index) {
        padded_capsules.set(index, robot_mask_capsules[index]);
    }
    material->set_shader_parameter("robot_mask_enabled", robot_mask_enabled && capsule_values >= 2);
    material->set_shader_parameter("robot_capsule_count", capsule_values / 2);
    material->set_shader_parameter("robot_capsules", padded_capsules);
}

void RealSenseSharedMemoryPointCloud::apply_direct_realsense_filter_settings() {
    if (direct_realsense.is_null()) {
        return;
    }
    direct_realsense->set_post_processing_enabled(direct_realsense_post_processing_enabled);
    direct_realsense->set_decimation_filter_enabled(direct_realsense_decimation_filter_enabled);
    direct_realsense->set_decimation_magnitude(direct_realsense_decimation_magnitude);
    direct_realsense->set_rotation_filter_enabled(direct_realsense_rotation_filter_enabled);
    direct_realsense->set_hdr_merge_filter_enabled(direct_realsense_hdr_merge_filter_enabled);
    direct_realsense->set_sequence_id_filter_enabled(direct_realsense_sequence_id_filter_enabled);
    direct_realsense->set_threshold_filter_enabled(direct_realsense_threshold_filter_enabled);
    direct_realsense->set_depth_to_disparity_filter_enabled(direct_realsense_depth_to_disparity_filter_enabled);
    direct_realsense->set_spatial_filter_enabled(direct_realsense_spatial_filter_enabled);
    direct_realsense->set_temporal_filter_enabled(direct_realsense_temporal_filter_enabled);
    direct_realsense->set_hole_filling_filter_enabled(direct_realsense_hole_filling_filter_enabled);
    direct_realsense->set_disparity_to_depth_filter_enabled(direct_realsense_disparity_to_depth_filter_enabled);
    direct_realsense->set_filter_depth_range(float(min_depth), float(max_depth));
    direct_realsense->set_hole_filling_mode(direct_realsense_hole_filling_mode);
}
