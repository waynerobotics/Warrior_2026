// multitask_engine.hpp
//
// TensorRT inference engine for the multitask perception model.
// Handles: Insta360 webcam frame → preprocess → GPU inference → FCOS decode + NMS → seg argmax
//
// Target platform: NVIDIA Jetson Orin, JetPack 5.x (TensorRT 8.5/8.6, CUDA 11.4+)
// Build deps: TensorRT, CUDA, OpenCV 4+

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime.h>
#include <NvInfer.h>
#include <opencv2/core.hpp>

namespace multitask {

// ─── Model constants ──────────────────────────────────────────────────────────

inline constexpr int INPUT_H     = 640;
inline constexpr int INPUT_W     = 1280;
inline constexpr int NUM_CLASSES = 6;
inline constexpr int PRED_DIM    = 11;   // 4 LTRB + 1 objectness + 6 class logits

// ImageNet normalisation (RGB order)
inline constexpr std::array<float, 3> IMAGENET_MEAN = {0.485f, 0.456f, 0.406f};
inline constexpr std::array<float, 3> IMAGENET_STD  = {0.229f, 0.224f, 0.225f};

inline constexpr const char* CLASS_NAMES[NUM_CLASSES] = {
    "barrel", "pedestrian", "stop_sign", "unknown", "tire", "pothole"
};

// ─── Public data types ────────────────────────────────────────────────────────

struct Detection {
    float x1, y1, x2, y2;
    float score;
    int   class_id;

    float cx()               const noexcept { return (x1 + x2) * 0.5f; }
    float cy()               const noexcept { return (y1 + y2) * 0.5f; }
    const char* class_name() const noexcept {
        return (class_id >= 0 && class_id < NUM_CLASSES)
               ? CLASS_NAMES[class_id] : "?";
    }
};

struct Config {
    float score_threshold = 0.20f;   // minimum objectness × class score to keep
    float nms_threshold   = 0.50f;   // class-agnostic IoU threshold for NMS
    int   max_detections  = 100;     // cap after NMS
    int   topk_per_level  = 100;     // pre-NMS cap per FPN level

    // Spatial filters (applied post-NMS, see postprocessing.md §2c / §3b-3c)
    float sky_zone_frac   = 0.35f;   // zero seg above this row fraction (sky)
    float robot_body_frac = 0.85f;   // drop det + zero seg below this row fraction
    float edge_excl_frac  = 0.05f;   // drop det in outer N% of image columns

    bool apply_morph      = false;   // optional 5×5 morphological close+open on seg mask
};

struct InferResult {
    std::vector<Detection> detections;
    cv::Mat                seg_mask;    // uint8 (H × W), 0 = background, 1 = lane
    double                 infer_ms;   // GPU kernel wall time
    double                 total_ms;   // full pipeline wall time (pre + infer + post)
};

// ─── TensorRT logger ──────────────────────────────────────────────────────────

class TRTLogger final : public nvinfer1::ILogger {
public:
    explicit TRTLogger(Severity min_level = Severity::kWARNING)
        : min_level_(min_level) {}

    void log(Severity severity, const char* msg) noexcept override;

private:
    Severity min_level_;
};

// ─── Inference engine ────────────────────────────────────────────────────────

class MultitaskEngine {
public:
    // Load a serialised TensorRT engine from `engine_path` (.trt / .engine file).
    // Throws std::runtime_error on any failure (bad path, wrong bindings, OOM).
    explicit MultitaskEngine(const std::string& engine_path,
                             Config cfg = Config{});
    ~MultitaskEngine();

    MultitaskEngine(const MultitaskEngine&)            = delete;
    MultitaskEngine& operator=(const MultitaskEngine&) = delete;
    MultitaskEngine(MultitaskEngine&&)                 = delete;
    MultitaskEngine& operator=(MultitaskEngine&&)      = delete;

    // Run the full pipeline on one BGR frame (any resolution — resized internally).
    InferResult infer(const cv::Mat& bgr_frame);

    const Config& config() const noexcept { return cfg_; }

private:
    // ── Initialisation ────────────────────────────────────────────────────────
    void load_engine(const std::string& path);
    void allocate_buffers();
    int  binding_index(const std::string& name) const;

    // ── Per-frame pipeline ────────────────────────────────────────────────────
    void    preprocess(const cv::Mat& bgr);

    // Decode one FPN level into `out` (appends — caller is responsible for clearing).
    void    decode_level(const float* feat, int feat_h, int feat_w,
                         std::vector<Detection>& out) const;

    void    run_nms(std::vector<Detection>& dets) const;
    void    apply_spatial_filters(std::vector<Detection>& dets) const;
    cv::Mat decode_segmentation(const float* logits) const;

    // ── Members ───────────────────────────────────────────────────────────────
    Config cfg_;

    TRTLogger                                    logger_;
    std::unique_ptr<nvinfer1::IRuntime>          runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine>        engine_;
    std::unique_ptr<nvinfer1::IExecutionContext>  context_;

    cudaStream_t stream_ = nullptr;

    // Device buffers
    void* d_input_  = nullptr;
    void* d_det_p3_ = nullptr;
    void* d_det_p4_ = nullptr;
    void* d_det_p5_ = nullptr;
    void* d_seg_    = nullptr;

    // Pinned host output buffers (zero-copy DMA-able memory)
    float* h_det_p3_ = nullptr;
    float* h_det_p4_ = nullptr;
    float* h_det_p5_ = nullptr;
    float* h_seg_    = nullptr;

    // CPU staging buffer for the normalised input tensor (reused each frame)
    std::vector<float> h_input_;

    // Binding indices, cached at construction time
    int idx_input_  = -1;
    int idx_det_p3_ = -1;
    int idx_det_p4_ = -1;
    int idx_det_p5_ = -1;
    int idx_seg_    = -1;
};

}  // namespace multitask
