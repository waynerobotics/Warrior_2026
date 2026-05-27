// main.cpp
//
// Live inference application for the multitask perception model.
// Captures frames from an Insta360 camera in webcam mode (or any V4L2 / video file),
// runs TensorRT inference, and displays annotated detections + lane segmentation.
//
// Usage:
//   ./multitask_infer --engine model.trt [OPTIONS]
//   ./multitask_infer --help

#include "multitask_engine.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// ─── Per-class overlay colours (BGR) ─────────────────────────────────────────

static const cv::Scalar CLASS_COLORS[multitask::NUM_CLASSES] = {
    { 0, 165, 255},   // barrel      — orange
    {255,  50,  50},  // pedestrian  — blue-ish
    { 50, 220,  50},  // stop_sign   — green
    {160, 160, 160},  // unknown     — grey
    { 50, 220, 220},  // tire        — yellow
    {220,  50, 220},  // pothole     — magenta
};

// ─── Graceful shutdown via SIGINT / SIGTERM ───────────────────────────────────

static std::atomic<bool> g_running{true};
static void handle_signal(int) noexcept { g_running.store(false); }

// ─── Visualisation helpers ────────────────────────────────────────────────────

// Blend the lane segmentation mask into `bgr` as a semi-transparent green tint.
static cv::Mat overlay_seg(const cv::Mat& bgr, const cv::Mat& mask) {
    cv::Mat out = bgr.clone();
    for (int y = 0; y < bgr.rows; ++y) {
        const uint8_t* m = mask.ptr<uint8_t>(y);
        cv::Vec3b*     d = out.ptr<cv::Vec3b>(y);
        for (int x = 0; x < bgr.cols; ++x) {
            if (m[x]) {
                // 75% original + 25% bright-green tint
                d[x][0] = static_cast<uint8_t>(d[x][0] * 0.75f);
                d[x][1] = static_cast<uint8_t>(d[x][1] * 0.75f + 220 * 0.25f);
                d[x][2] = static_cast<uint8_t>(d[x][2] * 0.75f + 80  * 0.25f);
            }
        }
    }
    return out;
}

// Draw all detection boxes with labelled class + score.
static void draw_detections(cv::Mat& img,
                             const std::vector<multitask::Detection>& dets)
{
    for (const auto& d : dets) {
        const int ci = std::clamp(d.class_id, 0, multitask::NUM_CLASSES - 1);
        const cv::Scalar& col = CLASS_COLORS[ci];

        cv::rectangle(img,
                      cv::Point2f(d.x1, d.y1),
                      cv::Point2f(d.x2, d.y2),
                      col, 2, cv::LINE_AA);

        char label[64];
        std::snprintf(label, sizeof(label), "%s  %.2f", d.class_name(), d.score);

        int baseline = 0;
        cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                      0.48, 1, &baseline);

        // Anchor the label just above the box top-left corner
        int lx = static_cast<int>(d.x1);
        int ly = static_cast<int>(d.y1) - baseline - 3;
        ly     = std::max(ly, ts.height + 2);

        cv::rectangle(img,
                      cv::Point(lx, ly - ts.height - 1),
                      cv::Point(lx + ts.width + 2, ly + baseline),
                      col, cv::FILLED);

        cv::putText(img, label, cv::Point(lx + 1, ly),
                    cv::FONT_HERSHEY_SIMPLEX, 0.48,
                    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
}

// Print timing and detection count into the top-left corner of the frame.
static void draw_hud(cv::Mat& img,
                     int n_dets, double infer_ms, double total_ms, int frame_idx)
{
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "Frame %-5d | Dets: %2d | GPU %.1f ms | Wall %.1f ms",
                  frame_idx, n_dets, infer_ms, total_ms);

    // Dark shadow + bright text for legibility on any background
    cv::putText(img, buf, cv::Point(8, 26),
                cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(img, buf, cv::Point(8, 26),
                cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
}

// ─── CLI ─────────────────────────────────────────────────────────────────────

struct Args {
    std::string engine_path;
    std::string source;           // empty → use camera_index; otherwise file / device path
    int         camera_index = 0;
    std::string save_path;        // if non-empty, write annotated video here
    float       score_thr    = 0.20f;
    float       nms_thr      = 0.50f;
    bool        apply_morph  = false;
    bool        no_display   = false;
    bool        verbose      = false;
};

static void print_usage(const char* prog) {
    std::printf(
        "Usage: %s --engine PATH [OPTIONS]\n"
        "\n"
        "Required:\n"
        "  --engine  PATH     TensorRT serialised engine (.trt / .engine)\n"
        "\n"
        "Input source (mutually exclusive; default: /dev/video0):\n"
        "  --camera  INT      V4L2 device index (default 0)\n"
        "  --source  PATH     Video file path, or /dev/videoN device\n"
        "\n"
        "Inference:\n"
        "  --score   FLOAT    Score threshold, default 0.20\n"
        "  --nms     FLOAT    NMS IoU threshold, default 0.50\n"
        "  --morph            Enable 5×5 morphological cleanup on the seg mask\n"
        "\n"
        "Output:\n"
        "  --save    PATH     Write annotated frames to an .mp4 file\n"
        "  --no-display       Suppress the OpenCV window (for headless systems)\n"
        "  --verbose          Print per-frame stats to stdout\n"
        "  --help             Show this message\n"
        "\n", prog);
}

static Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        auto require_next = [&]() -> std::string {
            if (i + 1 >= argc)
                throw std::runtime_error("Expected value after: " + arg);
            return argv[++i];
        };

        if      (arg == "--engine")     a.engine_path  = require_next();
        else if (arg == "--camera")     a.camera_index = std::stoi(require_next());
        else if (arg == "--source")     a.source       = require_next();
        else if (arg == "--save")       a.save_path    = require_next();
        else if (arg == "--score")      a.score_thr    = std::stof(require_next());
        else if (arg == "--nms")        a.nms_thr      = std::stof(require_next());
        else if (arg == "--morph")      a.apply_morph  = true;
        else if (arg == "--no-display") a.no_display   = true;
        else if (arg == "--verbose")    a.verbose      = true;
        else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (a.engine_path.empty()) {
        print_usage(argv[0]);
        throw std::runtime_error("--engine is required");
    }
    return a;
}

// ─── Camera open helper ───────────────────────────────────────────────────────

static cv::VideoCapture open_source(const Args& args) {
    cv::VideoCapture cap;

    if (!args.source.empty()) {
        // Numeric string → treat as device index; otherwise open as path
        bool all_digits = !args.source.empty() &&
                          std::all_of(args.source.begin(), args.source.end(),
                                      [](unsigned char c){ return std::isdigit(c); });
        if (all_digits)
            cap.open(std::stoi(args.source), cv::CAP_V4L2);
        else
            cap.open(args.source);
    } else {
        cap.open(args.camera_index, cv::CAP_V4L2);
    }

    if (!cap.isOpened())
        throw std::runtime_error("Cannot open video source");

    const bool is_camera = args.source.empty()
        || std::all_of(args.source.begin(), args.source.end(),
                       [](unsigned char c){ return std::isdigit(c); });

    if (is_camera) {
        // Request the Insta360's native equirectangular resolution.
        // The camera falls back to the closest supported mode if 3840×1920 is unavailable.
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  3840);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1920);
        cap.set(cv::CAP_PROP_FPS,          30);
        cap.set(cv::CAP_PROP_BUFFERSIZE,   1);   // minimise capture latency

        const double w   = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        const double h   = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        const double fps = cap.get(cv::CAP_PROP_FPS);
        std::printf("[INFO] Camera opened: %.0f × %.0f @ %.0f fps\n", w, h, fps);
    }

    return cap;
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) try {
    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    const Args args = parse_args(argc, argv);

    // ── Build engine ─────────────────────────────────────────────────────────
    multitask::Config cfg;
    cfg.score_threshold = args.score_thr;
    cfg.nms_threshold   = args.nms_thr;
    cfg.apply_morph     = args.apply_morph;

    std::printf("[INFO] Loading engine: %s\n", args.engine_path.c_str());
    multitask::MultitaskEngine engine(args.engine_path, cfg);
    std::printf("[INFO] Engine ready.\n");

    // ── Open input ───────────────────────────────────────────────────────────
    cv::VideoCapture cap = open_source(args);

    // ── Optional video writer ─────────────────────────────────────────────────
    cv::VideoWriter writer;
    if (!args.save_path.empty()) {
        const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        writer.open(args.save_path, fourcc, 30.0,
                    cv::Size(multitask::INPUT_W, multitask::INPUT_H), true);
        if (!writer.isOpened())
            throw std::runtime_error("Cannot open output file: " + args.save_path);
        std::printf("[INFO] Saving annotated video to: %s\n", args.save_path.c_str());
    }

    // ── Main loop ─────────────────────────────────────────────────────────────
    cv::Mat frame;
    int frame_idx = 0;

    std::printf("[INFO] Running — press 'q' or Ctrl-C to stop.\n");

    while (g_running.load()) {
        if (!cap.read(frame) || frame.empty()) {
            // Video file exhausted
            if (cap.get(cv::CAP_PROP_POS_FRAMES) > 0) break;
            std::fprintf(stderr, "[WARN] Empty frame on index %d, skipping.\n", frame_idx);
            continue;
        }

        const multitask::InferResult result = engine.infer(frame);

        // ── Build display frame ───────────────────────────────────────────────

        // Down-scale captured frame to model resolution for a 1:1 pixel overlay
        cv::Mat display;
        cv::resize(frame, display,
                   cv::Size(multitask::INPUT_W, multitask::INPUT_H),
                   0.0, 0.0, cv::INTER_LINEAR);

        display = overlay_seg(display, result.seg_mask);
        draw_detections(display, result.detections);
        draw_hud(display,
                 static_cast<int>(result.detections.size()),
                 result.infer_ms, result.total_ms, frame_idx);

        if (writer.isOpened())
            writer.write(display);

        if (!args.no_display) {
            cv::imshow("Multitask Perception — Insta360", display);
            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27 /* ESC */)
                break;
        }

        if (args.verbose || (frame_idx % 60 == 0)) {
            std::printf("[%5d] dets=%-3zu  GPU=%.1f ms  wall=%.1f ms\n",
                        frame_idx,
                        result.detections.size(),
                        result.infer_ms, result.total_ms);
            std::fflush(stdout);
        }

        ++frame_idx;
    }

    std::printf("[INFO] Stopped after %d frames.\n", frame_idx);
    return EXIT_SUCCESS;

} catch (const std::exception& ex) {
    std::fprintf(stderr, "[FATAL] %s\n", ex.what());
    return EXIT_FAILURE;
}
