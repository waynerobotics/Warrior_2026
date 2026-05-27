// multitask_engine.cpp
//
// TensorRT inference engine implementation.
// See include/multitask_engine.hpp and docs/postprocessing.md for the full spec.

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

// ─── Utility macros ───────────────────────────────────────────────────────────

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

// ─── File-local helpers ───────────────────────────────────────────────────────

namespace {

float sigmoid(float x) noexcept {
    return 1.f / (1.f + std::exp(-x));
}

// Numerically stable in-place softmax over a fixed-length array.
void softmax_inplace(float* arr, int n) noexcept {
    float max_v = arr[0];
    for (int i = 1; i < n; ++i) max_v = std::max(max_v, arr[i]);

    float sum = 0.f;
    for (int i = 0; i < n; ++i) {
        arr[i] = std::exp(arr[i] - max_v);
        sum += arr[i];
    }
    for (int i = 0; i < n; ++i) arr[i] /= sum;
}

// Class-agnostic IoU for greedy NMS.
float iou(const multitask::Detection& a, const multitask::Detection& b) noexcept {
    float ix1 = std::max(a.x1, b.x1);
    float iy1 = std::max(a.y1, b.y1);
    float ix2 = std::min(a.x2, b.x2);
    float iy2 = std::min(a.y2, b.y2);
    float inter = std::max(0.f, ix2 - ix1) * std::max(0.f, iy2 - iy1);
    if (inter == 0.f) return 0.f;
    float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    return inter / (area_a + area_b - inter + 1e-6f);
}

}  // namespace

namespace multitask {

// ─── TRTLogger ───────────────────────────────────────────────────────────────

void TRTLogger::log(Severity severity, const char* msg) noexcept {
    if (severity > min_level_) return;
    const char* tag = "[TRT]";
    switch (severity) {
        case Severity::kINTERNAL_ERROR: std::fprintf(stderr, "%s [E] %s\n", tag, msg); break;
        case Severity::kERROR:          std::fprintf(stderr, "%s [E] %s\n", tag, msg); break;
        case Severity::kWARNING:        std::fprintf(stderr, "%s [W] %s\n", tag, msg); break;
        case Severity::kINFO:           std::fprintf(stderr, "%s [I] %s\n", tag, msg); break;
        case Severity::kVERBOSE:        std::fprintf(stderr, "%s [V] %s\n", tag, msg); break;
    }
}

// ─── Construction / destruction ───────────────────────────────────────────────

MultitaskEngine::MultitaskEngine(const std::string& engine_path, Config cfg)
    : cfg_(std::move(cfg))
    , logger_(nvinfer1::ILogger::Severity::kWARNING)
{
    load_engine(engine_path);
    allocate_buffers();
}

MultitaskEngine::~MultitaskEngine() {
    // Order: context before engine, engine before runtime.
    context_.reset();
    engine_.reset();
    runtime_.reset();

    if (h_det_p3_) { cudaFreeHost(h_det_p3_); h_det_p3_ = nullptr; }
    if (h_det_p4_) { cudaFreeHost(h_det_p4_); h_det_p4_ = nullptr; }
    if (h_det_p5_) { cudaFreeHost(h_det_p5_); h_det_p5_ = nullptr; }
    if (h_seg_)    { cudaFreeHost(h_seg_);     h_seg_    = nullptr; }

    if (d_input_)  { cudaFree(d_input_);  d_input_  = nullptr; }
    if (d_det_p3_) { cudaFree(d_det_p3_); d_det_p3_ = nullptr; }
    if (d_det_p4_) { cudaFree(d_det_p4_); d_det_p4_ = nullptr; }
    if (d_det_p5_) { cudaFree(d_det_p5_); d_det_p5_ = nullptr; }
    if (d_seg_)    { cudaFree(d_seg_);    d_seg_    = nullptr; }

    if (stream_) { cudaStreamDestroy(stream_); stream_ = nullptr; }
}

// ─── Engine loading ───────────────────────────────────────────────────────────

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
        throw std::runtime_error("Failed to create TensorRT IRuntime");

    engine_.reset(runtime_->deserializeCudaEngine(buf.data(), buf.size()));
    if (!engine_)
        throw std::runtime_error("Failed to deserialise engine from: " + path);

    context_.reset(engine_->createExecutionContext());
    if (!context_)
        throw std::runtime_error("Failed to create IExecutionContext");

    idx_input_  = binding_index("input");
    idx_det_p3_ = binding_index("det_p3");
    idx_det_p4_ = binding_index("det_p4");
    idx_det_p5_ = binding_index("det_p5");
    idx_seg_    = binding_index("seg_logits");

    std::fprintf(stderr,
        "[multitask] Engine loaded from %s | bindings: in=%d p3=%d p4=%d p5=%d seg=%d\n",
        path.c_str(), idx_input_, idx_det_p3_, idx_det_p4_, idx_det_p5_, idx_seg_);
}

int MultitaskEngine::binding_index(const std::string& name) const {
    int idx = engine_->getBindingIndex(name.c_str());
    if (idx < 0)
        throw std::runtime_error("Binding not found in engine: \"" + name + "\"");
    return idx;
}

// ─── Buffer allocation ────────────────────────────────────────────────────────

void MultitaskEngine::allocate_buffers() {
    CUDA_CHECK(cudaStreamCreate(&stream_));

    // Fixed sizes derived from the model's output shapes (see postprocessing.md §1)
    constexpr size_t kInput  = 1 * 3 * INPUT_H * INPUT_W;        // 2 457 600
    constexpr size_t kP3     = 1 * PRED_DIM * 80 * 160;          //   140 800
    constexpr size_t kP4     = 1 * PRED_DIM * 40 * 80;           //    35 200
    constexpr size_t kP5     = 1 * PRED_DIM * 20 * 40;           //     8 800
    constexpr size_t kSeg    = 1 * 2 * INPUT_H * INPUT_W;        // 1 638 400

    CUDA_CHECK(cudaMalloc(&d_input_,  kInput * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_det_p3_, kP3    * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_det_p4_, kP4    * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_det_p5_, kP5    * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_seg_,    kSeg   * sizeof(float)));

    // Pinned host memory enables DMA transfers without an extra memcpy bounce.
    CUDA_CHECK(cudaMallocHost(&h_det_p3_, kP3  * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_det_p4_, kP4  * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_det_p5_, kP5  * sizeof(float)));
    CUDA_CHECK(cudaMallocHost(&h_seg_,    kSeg * sizeof(float)));

    h_input_.resize(kInput);
}

// ─── Preprocessing ───────────────────────────────────────────────────────────

void MultitaskEngine::preprocess(const cv::Mat& bgr) {
    // 1. Resize to model input resolution.
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(INPUT_W, INPUT_H), 0.0, 0.0, cv::INTER_LINEAR);

    // 2. BGR → RGB.
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    // 3. uint8 → float32 in [0, 1].
    rgb.convertTo(rgb, CV_32FC3, 1.f / 255.f);

    // 4. Split into three single-channel planes.
    std::vector<cv::Mat> planes(3);
    cv::split(rgb, planes);

    // 5. Normalise each channel and pack into CHW layout.
    //    convertTo with alpha = 1/std, beta = -mean/std  →  (x - mean) / std
    float* dst = h_input_.data();
    for (int c = 0; c < 3; ++c) {
        cv::Mat norm;
        planes[c].convertTo(norm, CV_32F,
                            1.f / IMAGENET_STD[c],
                            -IMAGENET_MEAN[c] / IMAGENET_STD[c]);
        std::memcpy(dst, norm.ptr<float>(), INPUT_H * INPUT_W * sizeof(float));
        dst += INPUT_H * INPUT_W;
    }

    // 6. Upload to GPU asynchronously.
    CUDA_CHECK(cudaMemcpyAsync(
        d_input_, h_input_.data(),
        h_input_.size() * sizeof(float),
        cudaMemcpyHostToDevice, stream_));
}

// ─── Inference ───────────────────────────────────────────────────────────────

InferResult MultitaskEngine::infer(const cv::Mat& bgr_frame) {
    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::duration<double, std::milli>;

    auto t0 = Clock::now();

    preprocess(bgr_frame);

    // Build the binding pointer array indexed by TRT binding index.
    // TRT 8.x: enqueueV2 with a flat array of device pointers.
    // TRT 10+: migrate to context->setInputTensorAddress / setOutputTensorAddress + executeV2.
    std::vector<void*> bindings(static_cast<size_t>(engine_->getNbBindings()), nullptr);
    bindings[idx_input_]  = d_input_;
    bindings[idx_det_p3_] = d_det_p3_;
    bindings[idx_det_p4_] = d_det_p4_;
    bindings[idx_det_p5_] = d_det_p5_;
    bindings[idx_seg_]    = d_seg_;

    auto t_gpu_start = Clock::now();
    if (!context_->enqueueV2(bindings.data(), stream_, nullptr))
        throw std::runtime_error("TensorRT enqueueV2 failed");
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    auto t_gpu_end = Clock::now();

    // Copy outputs back to pinned host buffers in a single pass.
    auto async_d2h = [this](void* dst, void* src, size_t bytes) {
        CUDA_CHECK(cudaMemcpyAsync(dst, src, bytes, cudaMemcpyDeviceToHost, stream_));
    };
    async_d2h(h_det_p3_, d_det_p3_, 1 * PRED_DIM * 80 * 160 * sizeof(float));
    async_d2h(h_det_p4_, d_det_p4_, 1 * PRED_DIM * 40 *  80 * sizeof(float));
    async_d2h(h_det_p5_, d_det_p5_, 1 * PRED_DIM * 20 *  40 * sizeof(float));
    async_d2h(h_seg_,    d_seg_,    1 * 2 * INPUT_H * INPUT_W * sizeof(float));
    CUDA_CHECK(cudaStreamSynchronize(stream_));

    // ── Post-processing (CPU) ─────────────────────────────────────────────────

    InferResult result;

    std::vector<Detection> all_dets;
    all_dets.reserve(cfg_.topk_per_level * 3);
    decode_level(h_det_p3_, 80, 160, all_dets);   // stride 8
    decode_level(h_det_p4_, 40,  80, all_dets);   // stride 16
    decode_level(h_det_p5_, 20,  40, all_dets);   // stride 32

    run_nms(all_dets);
    apply_spatial_filters(all_dets);
    result.detections = std::move(all_dets);

    result.seg_mask = decode_segmentation(h_seg_);

    auto t1 = Clock::now();
    result.infer_ms = Ms(t_gpu_end - t_gpu_start).count();
    result.total_ms = Ms(t1 - t0).count();

    return result;
}

// ─── FCOS detection decode ───────────────────────────────────────────────────

void MultitaskEngine::decode_level(
    const float* feat, int feat_h, int feat_w,
    std::vector<Detection>& out) const
{
    const float stride_y  = static_cast<float>(INPUT_H) / feat_h;
    const float stride_x  = static_cast<float>(INPUT_W) / feat_w;
    const int   cell_cnt  = feat_h * feat_w;

    // feat layout: (1, PRED_DIM, feat_h, feat_w) in row-major (C-contiguous).
    // Channel offsets:
    const float* box_base = feat;                  // channels 0–3: LTRB
    const float* obj_base = feat + 4 * cell_cnt;   // channel  4:   objectness logit
    const float* cls_base = feat + 5 * cell_cnt;   // channels 5–10: class logits

    std::vector<Detection> level_dets;
    level_dets.reserve(256);

    for (int i = 0; i < feat_h; ++i) {
        for (int j = 0; j < feat_w; ++j) {
            const int k = i * feat_w + j;

            // Cell-centre coordinates (half-pixel offset)
            const float cx = (j + 0.5f) * stride_x;
            const float cy = (i + 0.5f) * stride_y;

            // ReLU-clamped LTRB offsets scaled back to pixel space
            const float l = std::max(0.f, box_base[0 * cell_cnt + k]) * stride_x;
            const float t = std::max(0.f, box_base[1 * cell_cnt + k]) * stride_y;
            const float r = std::max(0.f, box_base[2 * cell_cnt + k]) * stride_x;
            const float b = std::max(0.f, box_base[3 * cell_cnt + k]) * stride_y;

            const float obj = sigmoid(obj_base[k]);

            // Softmax over the 6 class logits, then argmax
            float cls[NUM_CLASSES];
            for (int c = 0; c < NUM_CLASSES; ++c)
                cls[c] = cls_base[c * cell_cnt + k];
            softmax_inplace(cls, NUM_CLASSES);

            int   best_cls = 0;
            float best_p   = cls[0];
            for (int c = 1; c < NUM_CLASSES; ++c) {
                if (cls[c] > best_p) { best_p = cls[c]; best_cls = c; }
            }

            const float score = obj * best_p;
            if (score < cfg_.score_threshold) continue;

            Detection d;
            d.x1       = std::clamp(cx - l, 0.f, static_cast<float>(INPUT_W));
            d.y1       = std::clamp(cy - t, 0.f, static_cast<float>(INPUT_H));
            d.x2       = std::clamp(cx + r, 0.f, static_cast<float>(INPUT_W));
            d.y2       = std::clamp(cy + b, 0.f, static_cast<float>(INPUT_H));
            d.score    = score;
            d.class_id = best_cls;
            level_dets.push_back(d);
        }
    }

    // Per-level topk cap (keeps memory bounded before the global NMS pass)
    if (static_cast<int>(level_dets.size()) > cfg_.topk_per_level) {
        std::partial_sort(
            level_dets.begin(),
            level_dets.begin() + cfg_.topk_per_level,
            level_dets.end(),
            [](const Detection& a, const Detection& b) { return a.score > b.score; });
        level_dets.resize(cfg_.topk_per_level);
    }

    out.insert(out.end(), level_dets.begin(), level_dets.end());
}

// ─── Class-agnostic greedy NMS ────────────────────────────────────────────────

void MultitaskEngine::run_nms(std::vector<Detection>& dets) const {
    std::sort(dets.begin(), dets.end(),
              [](const Detection& a, const Detection& b) { return a.score > b.score; });

    std::vector<bool> suppressed(dets.size(), false);
    std::vector<Detection> kept;
    kept.reserve(std::min(static_cast<int>(dets.size()), cfg_.max_detections));

    for (size_t i = 0; i < dets.size(); ++i) {
        if (suppressed[i]) continue;
        kept.push_back(dets[i]);
        if (static_cast<int>(kept.size()) >= cfg_.max_detections) break;

        for (size_t j = i + 1; j < dets.size(); ++j)
            if (!suppressed[j] && iou(dets[i], dets[j]) > cfg_.nms_threshold)
                suppressed[j] = true;
    }

    dets = std::move(kept);
}

// ─── Spatial filters (postprocessing.md §2c) ─────────────────────────────────

void MultitaskEngine::apply_spatial_filters(std::vector<Detection>& dets) const {
    const float body_row    = cfg_.robot_body_frac * INPUT_H;
    const float edge_lo     = cfg_.edge_excl_frac * INPUT_W;
    const float edge_hi     = (1.f - cfg_.edge_excl_frac) * INPUT_W;

    dets.erase(std::remove_if(dets.begin(), dets.end(),
        [body_row, edge_lo, edge_hi](const Detection& d) {
            const float cx = d.cx(), cy = d.cy();
            return cy >= body_row                   // robot body (bottom 15%)
                || cx < edge_lo || cx > edge_hi;   // equirectangular seam
        }),
        dets.end());
}

// ─── Segmentation decode (postprocessing.md §3) ──────────────────────────────

cv::Mat MultitaskEngine::decode_segmentation(const float* logits) const {
    // logits layout: (1, 2, H, W) — channel 0 is background, channel 1 is lane
    const int   total     = INPUT_H * INPUT_W;
    const float* bg_plane  = logits;
    const float* lane_plane = logits + total;

    cv::Mat mask(INPUT_H, INPUT_W, CV_8UC1);
    uint8_t* dst = mask.ptr<uint8_t>();
    for (int i = 0; i < total; ++i)
        dst[i] = (lane_plane[i] > bg_plane[i]) ? 1u : 0u;

    // Sky zone — top 35% of equirectangular frame is sky
    const int sky_row  = static_cast<int>(cfg_.sky_zone_frac   * INPUT_H);
    const int body_row = static_cast<int>(cfg_.robot_body_frac * INPUT_H);
    mask.rowRange(0,        sky_row).setTo(0);
    mask.rowRange(body_row, INPUT_H).setTo(0);

    if (cfg_.apply_morph) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    }

    return mask;
}

}  // namespace multitask
