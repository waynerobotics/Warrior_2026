// multitask_engine.cpp
//
// TensorRT inference engine implementation.
// TensorRT 10.x compatible version for JetPack 6 / CUDA 12.x

#include "multitask_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <numeric>
#include <stdexcept>

#include <cuda_runtime.h>
#include <opencv2/imgproc.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// Utility macros
// ─────────────────────────────────────────────────────────────────────────────

#define CUDA_CHECK(call)                                                      \
    do {                                                                      \
        cudaError_t _e = (call);                                              \
        if (_e != cudaSuccess) {                                              \
            throw std::runtime_error(                                         \
                std::string("CUDA error at ") + __FILE__ + ":"               \
                + std::to_string(__LINE__) + " — "                           \
                + cudaGetErrorString(_e));                                    \
        }                                                                     \
    } while (0)

// ─────────────────────────────────────────────────────────────────────────────
// File-local helpers
// ─────────────────────────────────────────────────────────────────────────────

namespace {

float sigmoid(float x) noexcept {
    return 1.f / (1.f + std::exp(-x));
}

float iou(const multitask::Detection& a,
          const multitask::Detection& b) noexcept
{
    float ix1 = std::max(a.x1, b.x1);
    float iy1 = std::max(a.y1, b.y1);
    float ix2 = std::min(a.x2, b.x2);
    float iy2 = std::min(a.y2, b.y2);

    float inter =
        std::max(0.f, ix2 - ix1) *
        std::max(0.f, iy2 - iy1);

    if (inter == 0.f)
        return 0.f;

    float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);

    return inter / (area_a + area_b - inter + 1e-6f);
}

} // namespace

namespace multitask {

// ─────────────────────────────────────────────────────────────────────────────
// TRTLogger
// ─────────────────────────────────────────────────────────────────────────────

void TRTLogger::log(Severity severity, const char* msg) noexcept {
    if (severity > min_level_)
        return;

    const char* tag = "[TRT]";

    switch (severity) {
        case Severity::kINTERNAL_ERROR:
        case Severity::kERROR:
            std::fprintf(stderr, "%s [E] %s\n", tag, msg);
            break;
        case Severity::kWARNING:
            std::fprintf(stderr, "%s [W] %s\n", tag, msg);
            break;
        case Severity::kINFO:
            std::fprintf(stderr, "%s [I] %s\n", tag, msg);
            break;
        case Severity::kVERBOSE:
            std::fprintf(stderr, "%s [V] %s\n", tag, msg);
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Construction / destruction
// ─────────────────────────────────────────────────────────────────────────────

MultitaskEngine::MultitaskEngine(
    const std::string& engine_path,
    Config cfg)
    : cfg_(std::move(cfg))
    , logger_(nvinfer1::ILogger::Severity::kWARNING)
{
    load_engine(engine_path);
    allocate_buffers();
}

MultitaskEngine::~MultitaskEngine() {

    context_.reset();
    engine_.reset();
    runtime_.reset();

    if (h_det_p3_) cudaFreeHost(h_det_p3_);
    if (h_det_p4_) cudaFreeHost(h_det_p4_);
    if (h_det_p5_) cudaFreeHost(h_det_p5_);
    if (h_seg_)    cudaFreeHost(h_seg_);

    if (d_input_)  cudaFree(d_input_);
    if (d_det_p3_) cudaFree(d_det_p3_);
    if (d_det_p4_) cudaFree(d_det_p4_);
    if (d_det_p5_) cudaFree(d_det_p5_);
    if (d_seg_)    cudaFree(d_seg_);

    if (stream_)
        cudaStreamDestroy(stream_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Engine loading
// ─────────────────────────────────────────────────────────────────────────────

void MultitaskEngine::load_engine(const std::string& path) {

    std::ifstream file(path, std::ios::binary | std::ios::ate);

    if (!file.is_open())
        throw std::runtime_error("Cannot open engine file: " + path);

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buf(static_cast<size_t>(size));

    if (!file.read(buf.data(), size))
        throw std::runtime_error("Failed to read engine file: " + path);

    runtime_.reset(nvinfer1::createInferRuntime(logger_));

    if (!runtime_)
        throw std::runtime_error("Failed to create TensorRT runtime");

    engine_.reset(runtime_->deserializeCudaEngine(buf.data(), buf.size()));

    if (!engine_)
        throw std::runtime_error("Failed to deserialize TensorRT engine");

    context_.reset(engine_->createExecutionContext());

    if (!context_)
        throw std::runtime_error("Failed to create execution context");

    // TensorRT 10.x tensor indices
    idx_input_  = binding_index("input");
    idx_det_p3_ = binding_index("det_p3");
    idx_det_p4_ = binding_index("det_p4");
    idx_det_p5_ = binding_index("det_p5");
    idx_seg_    = binding_index("seg_logits");

    std::fprintf(stderr, "\n=== TensorRT IO Tensors ===\n");
    for (int i = 0; i < engine_->getNbIOTensors(); ++i)
        std::fprintf(stderr, "[%d] %s\n", i, engine_->getIOTensorName(i));
    std::fprintf(stderr, "===========================\n\n");

    std::fprintf(stderr, "[multitask] Engine loaded from %s\n", path.c_str());
}

int MultitaskEngine::binding_index(const std::string& name) const {

    const int nb = engine_->getNbIOTensors();

    for (int i = 0; i < nb; ++i) {
        if (name == engine_->getIOTensorName(i))
            return i;
    }

    throw std::runtime_error("Tensor not found in engine: " + name);
}

// ─────────────────────────────────────────────────────────────────────────────
// Buffer allocation
// ─────────────────────────────────────────────────────────────────────────────

void MultitaskEngine::allocate_buffers() {

    CUDA_CHECK(cudaStreamCreate(&stream_));

    constexpr size_t kInput = 1 * 3   * INPUT_H * INPUT_W;
    constexpr size_t kP3    = 1 * PRED_DIM * 80  * 160;
    constexpr size_t kP4    = 1 * PRED_DIM * 40  * 80;
    constexpr size_t kP5    = 1 * PRED_DIM * 20  * 40;
    constexpr size_t kSeg   = 1 * 2   * INPUT_H * INPUT_W;

    CUDA_CHECK(cudaMalloc(&d_input_,  kInput * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_det_p3_, kP3    * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_det_p4_, kP4    * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_det_p5_, kP5    * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_seg_,    kSeg   * sizeof(float)));

    CUDA_CHECK(cudaMallocHost(&h_det_p3_, kP3  * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_det_p4_, kP4  * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_det_p5_, kP5  * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_seg_,    kSeg * sizeof(float)));

    h_input_.resize(kInput);
}

// ─────────────────────────────────────────────────────────────────────────────
// Preprocessing
// ─────────────────────────────────────────────────────────────────────────────

void MultitaskEngine::preprocess(const cv::Mat& bgr) {

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(INPUT_W, INPUT_H));

    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32FC3, 1.f / 255.f);

    std::vector<cv::Mat> planes(3);
    cv::split(rgb, planes);

    float* dst = h_input_.data();

    for (int c = 0; c < 3; ++c) {
        cv::Mat norm;
        planes[c].convertTo(
            norm, CV_32F,
            1.f / IMAGENET_STD[c],
            -IMAGENET_MEAN[c] / IMAGENET_STD[c]);

        std::memcpy(dst, norm.ptr<float>(), INPUT_H * INPUT_W * sizeof(float));
        dst += INPUT_H * INPUT_W;
    }

    CUDA_CHECK(cudaMemcpyAsync(
        d_input_,
        h_input_.data(),
        h_input_.size() * sizeof(float),
        cudaMemcpyHostToDevice,
        stream_));
}

// ─────────────────────────────────────────────────────────────────────────────
// Inference
// ─────────────────────────────────────────────────────────────────────────────

InferResult MultitaskEngine::infer(const cv::Mat& bgr_frame) {

    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::duration<double, std::milli>;

    auto t0 = Clock::now();

    preprocess(bgr_frame);

    // Set input shape (required for dynamic engines)
    context_->setInputShape("input", nvinfer1::Dims4(1, 3, INPUT_H, INPUT_W));

    // Build binding table
    std::vector<void*> bindings(
        static_cast<size_t>(engine_->getNbIOTensors()), nullptr);

    bindings[idx_input_]  = d_input_;
    bindings[idx_det_p3_] = d_det_p3_;
    bindings[idx_det_p4_] = d_det_p4_;
    bindings[idx_det_p5_] = d_det_p5_;
    bindings[idx_seg_]    = d_seg_;

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
        const char* name = engine_->getIOTensorName(i);
        if (!context_->setTensorAddress(name, bindings[i]))
            throw std::runtime_error(
                std::string("Failed to set tensor address: ") + name);
    }

    auto t_gpu_start = Clock::now();

    if (!context_->enqueueV3(stream_))
        throw std::runtime_error("TensorRT enqueueV3 failed");

    CUDA_CHECK(cudaStreamSynchronize(stream_));

    auto t_gpu_end = Clock::now();

    // Async device → host copies
    auto async_d2h = [this](void* dst, void* src, size_t bytes) {
        CUDA_CHECK(cudaMemcpyAsync(dst, src, bytes, cudaMemcpyDeviceToHost, stream_));
    };

    async_d2h(h_det_p3_, d_det_p3_, PRED_DIM * 80  * 160 * sizeof(float));
    async_d2h(h_det_p4_, d_det_p4_, PRED_DIM * 40  * 80  * sizeof(float));
    async_d2h(h_det_p5_, d_det_p5_, PRED_DIM * 20  * 40  * sizeof(float));
    async_d2h(h_seg_,    d_seg_,    2        * INPUT_H * INPUT_W * sizeof(float));

    CUDA_CHECK(cudaStreamSynchronize(stream_));

    // Decode all FPN levels
    InferResult result;
    std::vector<Detection> all_dets;
    all_dets.reserve(cfg_.topk_per_level * 3);

    decode_level(h_det_p3_, 80, 160, all_dets);
    decode_level(h_det_p4_, 40, 80,  all_dets);
    decode_level(h_det_p5_, 20, 40,  all_dets);

    run_nms(all_dets);
    apply_spatial_filters(all_dets);

    // Cap to max_detections
    if (static_cast<int>(all_dets.size()) > cfg_.max_detections)
        all_dets.resize(static_cast<size_t>(cfg_.max_detections));

    result.detections = std::move(all_dets);
    result.seg_mask   = decode_segmentation(h_seg_);

    auto t1 = Clock::now();
    result.infer_ms = Ms(t_gpu_end - t_gpu_start).count();
    result.total_ms = Ms(t1 - t0).count();

    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// decode_level
//
// Tensor layout (channel-first): [1, PRED_DIM, feat_h, feat_w]
// PRED_DIM = 11: channels [0–3] = LTRB offsets, [4] = objectness, [5–10] = class logits
//
// FCOS-style decoding:
//   x1 = (gx + 0.5) * stride_w − sigmoid(l) * stride_w  [left distance]
//   y1 = (gy + 0.5) * stride_h − sigmoid(t) * stride_h  [top  distance]
//   x2 = (gx + 0.5) * stride_w + sigmoid(r) * stride_w  [right  distance]
//   y2 = (gy + 0.5) * stride_h + sigmoid(b) * stride_h  [bottom distance]
//
// If your model uses YOLO-style anchor-free (cx/cy/w/h) instead:
//   cx = (gx + sigmoid(ch[0])) * stride_w
//   cy = (gy + sigmoid(ch[1])) * stride_h
//   bw = exp(ch[2]) * stride_w
//   bh = exp(ch[3]) * stride_h
// ─────────────────────────────────────────────────────────────────────────────

void MultitaskEngine::decode_level(
    const float*            feat,
    int                     feat_h,
    int                     feat_w,
    std::vector<Detection>& out) const
{
    const float stride_h  = static_cast<float>(INPUT_H) / static_cast<float>(feat_h);
    const float stride_w  = static_cast<float>(INPUT_W) / static_cast<float>(feat_w);
    const int   num_cells = feat_h * feat_w;

    int top_count = 0;

    for (int cell = 0;
         cell < num_cells && top_count < cfg_.topk_per_level;
         ++cell)
    {
        const int gy = cell / feat_w;
        const int gx = cell % feat_w;

        // Channel accessor: channel-first layout
        auto ch = [&](int c) -> float {
            return feat[c * num_cells + cell];
        };

        const float obj_conf = sigmoid(ch(4));

        if (obj_conf < cfg_.score_threshold)
            continue;

        // Find best class among NUM_CLASSES logits
        int   best_cls   = 0;
        float best_logit = ch(5);

        for (int c = 1; c < NUM_CLASSES; ++c) {
            const float logit = ch(5 + c);
            if (logit > best_logit) {
                best_logit = logit;
                best_cls   = c;
            }
        }

        const float cls_score = obj_conf * sigmoid(best_logit);

        if (cls_score < cfg_.score_threshold)
            continue;

        // FCOS LTRB decoding — centre of grid cell ± predicted distances
        const float cx = (static_cast<float>(gx) + 0.5f) * stride_w;
        const float cy = (static_cast<float>(gy) + 0.5f) * stride_h;

        Detection d{};
        d.x1      = cx - sigmoid(ch(0)) * stride_w;   // left
        d.y1      = cy - sigmoid(ch(1)) * stride_h;   // top
        d.x2      = cx + sigmoid(ch(2)) * stride_w;   // right
        d.y2      = cy + sigmoid(ch(3)) * stride_h;   // bottom
        d.score    = cls_score;
        d.class_id = best_cls;

        // Clamp to image bounds
        d.x1 = std::max(0.f,                     d.x1);
        d.y1 = std::max(0.f,                     d.y1);
        d.x2 = std::min(static_cast<float>(INPUT_W), d.x2);
        d.y2 = std::min(static_cast<float>(INPUT_H), d.y2);

        out.push_back(d);
        ++top_count;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// run_nms
//
// Greedy NMS: sort by score descending, suppress overlapping same-class boxes
// whose IoU exceeds cfg_.nms_threshold.
// ─────────────────────────────────────────────────────────────────────────────

void MultitaskEngine::run_nms(std::vector<Detection>& dets) const {

    std::sort(
        dets.begin(), dets.end(),
        [](const Detection& a, const Detection& b) {
            return a.score > b.score;
        });

    const size_t n = dets.size();
    std::vector<bool> suppressed(n, false);

    for (size_t i = 0; i < n; ++i) {

        if (suppressed[i]) continue;

        for (size_t j = i + 1; j < n; ++j) {

            if (suppressed[j])                          continue;
            if (dets[i].class_id != dets[j].class_id)  continue;

            if (iou(dets[i], dets[j]) > cfg_.nms_threshold)
                suppressed[j] = true;
        }
    }

    // Compact in-place
    size_t write = 0;
    for (size_t i = 0; i < n; ++i)
        if (!suppressed[i])
            dets[write++] = dets[i];

    dets.resize(write);
}

// ─────────────────────────────────────────────────────────────────────────────
// apply_spatial_filters
//
// Implements postprocessing.md §2c / §3b-3c:
//   • Drop detections whose centre is in the sky zone (top sky_zone_frac rows)
//   • Drop detections whose centre is below the robot body zone
//     (bottom robot_body_frac row cutoff)
//   • Drop detections whose centre falls in the outer edge_excl_frac columns
// ─────────────────────────────────────────────────────────────────────────────

void MultitaskEngine::apply_spatial_filters(std::vector<Detection>& dets) const {

    const float sky_y    = cfg_.sky_zone_frac    * static_cast<float>(INPUT_H);
    const float body_y   = cfg_.robot_body_frac  * static_cast<float>(INPUT_H);
    const float edge_x   = cfg_.edge_excl_frac   * static_cast<float>(INPUT_W);
    const float right_x  = static_cast<float>(INPUT_W) - edge_x;

    dets.erase(
        std::remove_if(
            dets.begin(), dets.end(),
            [&](const Detection& d) {
                // Drop zero/negative area boxes
                if ((d.x2 - d.x1) <= 0.f || (d.y2 - d.y1) <= 0.f)
                    return true;

                const float cx = d.cx();
                const float cy = d.cy();

                if (cy < sky_y)   return true;   // sky zone
                if (cy > body_y)  return true;   // robot body zone
                if (cx < edge_x)  return true;   // left edge
                if (cx > right_x) return true;   // right edge

                return false;
            }),
        dets.end());
}

// ─────────────────────────────────────────────────────────────────────────────
// decode_segmentation
//
// Tensor layout: [1, 2, INPUT_H, INPUT_W] — channel-first.
//   Channel 0 = background logit
//   Channel 1 = lane (foreground) logit
//
// Output: CV_8UC1, values 0 (background) or 1 (lane), matching the
// InferResult::seg_mask contract in the header.
//
// Optionally applies a 5×5 morphological close+open if cfg_.apply_morph is set.
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat MultitaskEngine::decode_segmentation(const float* logits) const {

    const int num_pixels = INPUT_H * INPUT_W;

    cv::Mat mask(INPUT_H, INPUT_W, CV_8UC1);

    for (int px = 0; px < num_pixels; ++px) {
        const float bg = logits[px];
        const float fg = logits[num_pixels + px];
        mask.data[px] = static_cast<uint8_t>(fg > bg ? 1 : 0);
    }

    // Zero out sky zone
    const int sky_rows =
        static_cast<int>(cfg_.sky_zone_frac * static_cast<float>(INPUT_H));

    if (sky_rows > 0)
        mask.rowRange(0, std::min(sky_rows, INPUT_H)).setTo(0);

    // Zero out robot body zone
    const int body_row =
        static_cast<int>(cfg_.robot_body_frac * static_cast<float>(INPUT_H));

    if (body_row < INPUT_H)
        mask.rowRange(std::max(0, body_row), INPUT_H).setTo(0);

    // Optional morphological clean-up (5×5 close then open)
    if (cfg_.apply_morph) {
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(5, 5));

        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    }

    return mask;
}

} // namespace multitask