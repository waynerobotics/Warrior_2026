// infer_images.cpp
//
// Batch image inference for the multitask perception model.
// Accepts a manifest JSON (data/test_manifest.json), an image directory,
// or explicit image paths.
//
// Usage:
//   ./infer_images --engine model.trt --manifest data/test_manifest.json
//   ./infer_images --engine model.trt --image_dir data/frames/
//   ./infer_images --engine model.trt --image img1.jpg img2.jpg
//   ./infer_images --help

#include "multitask_engine.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using json = nlohmann::json;
namespace fs = std::filesystem;

// ─── Per-class overlay colours (BGR, matches main.cpp) ────────────────────────

static const cv::Scalar CLASS_COLORS[multitask::NUM_CLASSES] = {
    { 0, 165, 255},   // barrel      — orange
    {255,  50,  50},  // pedestrian  — blue-ish
    { 50, 220,  50},  // stop_sign   — green
    {160, 160, 160},  // unknown     — grey
    { 50, 220, 220},  // tire        — yellow
    {220,  50, 220},  // pothole     — magenta
};
static constexpr cv::Scalar GT_COLOR  = {80, 80, 220};   // dashed red for GT boxes
static constexpr cv::Scalar SEG_TINT  = {80, 220, 80};   // green lane tint

// ─── Manifest entry ───────────────────────────────────────────────────────────

struct ManifestEntry {
    fs::path                           image_path;
    std::vector<std::array<float, 4>>  gt_boxes;   // [x1, y1, x2, y2] in original image coords
    std::vector<int>                   gt_labels;
    fs::path                           gt_seg_path;
    bool                               has_gt = false;
};

// ─── Visualisation helpers ────────────────────────────────────────────────────

static cv::Mat overlay_seg(const cv::Mat& bgr, const cv::Mat& mask) {
    cv::Mat out = bgr.clone();
    for (int y = 0; y < bgr.rows; ++y) {
        const uint8_t* m = mask.ptr<uint8_t>(y);
        cv::Vec3b*     d = out.ptr<cv::Vec3b>(y);
        for (int x = 0; x < bgr.cols; ++x) {
            if (m[x]) {
                d[x][0] = static_cast<uint8_t>(d[x][0] * 0.75f);
                d[x][1] = static_cast<uint8_t>(d[x][1] * 0.75f + 220 * 0.25f);
                d[x][2] = static_cast<uint8_t>(d[x][2] * 0.75f +  80 * 0.25f);
            }
        }
    }
    return out;
}

static void draw_labeled_box(cv::Mat& img,
                              float x1, float y1, float x2, float y2,
                              const cv::Scalar& col,
                              const std::string& label,
                              int thickness = 2)
{
    cv::rectangle(img, cv::Point2f(x1, y1), cv::Point2f(x2, y2),
                  col, thickness, cv::LINE_AA);

    if (label.empty()) return;

    int baseline = 0;
    cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
    int lx = static_cast<int>(x1);
    int ly = static_cast<int>(y1) - baseline - 3;
    ly     = std::max(ly, ts.height + 2);

    cv::rectangle(img,
                  cv::Point(lx, ly - ts.height - 1),
                  cv::Point(lx + ts.width + 2, ly + baseline),
                  col, cv::FILLED);
    cv::putText(img, label, cv::Point(lx + 1, ly),
                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

static void draw_predictions(cv::Mat& img,
                              const std::vector<multitask::Detection>& dets)
{
    for (const auto& d : dets) {
        const int ci = std::clamp(d.class_id, 0, multitask::NUM_CLASSES - 1);
        char lbl[64];
        std::snprintf(lbl, sizeof(lbl), "%s %.2f", d.class_name(), d.score);
        draw_labeled_box(img, d.x1, d.y1, d.x2, d.y2, CLASS_COLORS[ci], lbl, 2);
    }
}

// GT boxes are drawn thinner and with a "GT:" prefix so they are visually distinct.
static void draw_gt_boxes(cv::Mat& img,
                           const std::vector<std::array<float, 4>>& boxes,
                           const std::vector<int>& labels)
{
    for (size_t i = 0; i < boxes.size(); ++i) {
        const int ci = (i < labels.size()) ? labels[i] : -1;
        const char* cn = (ci >= 0 && ci < multitask::NUM_CLASSES)
                         ? multitask::CLASS_NAMES[ci] : "?";
        std::string lbl = std::string("GT:") + cn;
        draw_labeled_box(img,
                         boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3],
                         GT_COLOR, lbl, 1);
    }
}

// ─── Manifest loader ──────────────────────────────────────────────────────────

static std::vector<ManifestEntry> load_manifest(const fs::path& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open manifest: " + path.string());

    json j;
    f >> j;

    const fs::path base = path.parent_path();
    std::vector<ManifestEntry> entries;

    for (const auto& item : j) {
        ManifestEntry e;
        e.image_path = fs::weakly_canonical(base / item["image"].get<std::string>());

        if (item.contains("boxes") && item.contains("labels")) {
            e.has_gt = true;
            for (const auto& box : item["boxes"]) {
                e.gt_boxes.push_back({
                    box[0].get<float>(), box[1].get<float>(),
                    box[2].get<float>(), box[3].get<float>()
                });
            }
            for (const auto& lbl : item["labels"])
                e.gt_labels.push_back(lbl.get<int>());
        }

        if (item.contains("seg_mask") && !item["seg_mask"].is_null())
            e.gt_seg_path = fs::weakly_canonical(base / item["seg_mask"].get<std::string>());

        entries.push_back(std::move(e));
    }
    return entries;
}

// ─── Scale GT boxes from original image resolution to model input size ─────────

static std::vector<std::array<float, 4>> scale_gt_boxes(
    const std::vector<std::array<float, 4>>& boxes,
    int orig_w, int orig_h)
{
    const float sx = static_cast<float>(multitask::INPUT_W) / orig_w;
    const float sy = static_cast<float>(multitask::INPUT_H) / orig_h;
    auto scaled = boxes;
    for (auto& b : scaled) {
        b[0] *= sx; b[2] *= sx;
        b[1] *= sy; b[3] *= sy;
    }
    return scaled;
}

// ─── CLI ─────────────────────────────────────────────────────────────────────

struct Args {
    std::string              engine_path;
    std::string              manifest_path;
    std::string              image_dir;
    std::vector<std::string> image_paths;
    std::string              output_dir  = "infer_out";
    float                    score_thr   = 0.20f;
    float                    nms_thr     = 0.50f;
    bool                     show_gt     = false;
    bool                     apply_morph = false;
    bool                     save_json   = false;
};

static void print_usage(const char* prog) {
    std::printf(
        "Usage: %s --engine PATH [SOURCE] [OPTIONS]\n"
        "\n"
        "Required:\n"
        "  --engine   PATH      TensorRT .trt / .engine file\n"
        "\n"
        "Source (pick one):\n"
        "  --manifest PATH      JSON manifest (e.g. data/test_manifest.json)\n"
        "  --image_dir PATH     Directory — all .jpg/.png inside are processed\n"
        "  --image    PATH...   One or more explicit image paths\n"
        "\n"
        "Options:\n"
        "  --output_dir PATH    Destination for annotated images (default: infer_out/)\n"
        "  --score    FLOAT     Score threshold (default 0.20)\n"
        "  --nms      FLOAT     NMS IoU threshold (default 0.50)\n"
        "  --show_gt            Draw ground-truth boxes (manifest source only)\n"
        "  --morph              Morphological cleanup on segmentation mask\n"
        "  --save_json          Write results.json to output_dir\n"
        "  --help               Show this message\n"
        "\n", prog);
}

static Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Expected value after: " + arg);
            return argv[++i];
        };

        if      (arg == "--engine")     a.engine_path   = next();
        else if (arg == "--manifest")   a.manifest_path = next();
        else if (arg == "--image_dir")  a.image_dir     = next();
        else if (arg == "--image") {
            // Consume every following non-flag token as an image path
            while (i + 1 < argc && argv[i + 1][0] != '-')
                a.image_paths.push_back(argv[++i]);
            if (a.image_paths.empty())
                throw std::runtime_error("--image requires at least one path");
        }
        else if (arg == "--output_dir") a.output_dir   = next();
        else if (arg == "--score")      a.score_thr    = std::stof(next());
        else if (arg == "--nms")        a.nms_thr      = std::stof(next());
        else if (arg == "--show_gt")    a.show_gt      = true;
        else if (arg == "--morph")      a.apply_morph  = true;
        else if (arg == "--save_json")  a.save_json    = true;
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

// ─── Build entry list from different sources ──────────────────────────────────

static constexpr std::array<const char*, 6> IMAGE_EXTS = {
    ".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"
};

static std::vector<ManifestEntry> collect_entries(const Args& args) {
    if (!args.manifest_path.empty()) {
        auto entries = load_manifest(args.manifest_path);
        std::printf("[INFO] Loaded %zu entries from manifest: %s\n",
                    entries.size(), args.manifest_path.c_str());
        return entries;
    }

    std::vector<ManifestEntry> entries;

    if (!args.image_dir.empty()) {
        for (const auto& p : fs::directory_iterator(args.image_dir)) {
            if (!p.is_regular_file()) continue;
            std::string ext = p.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            bool valid = std::any_of(IMAGE_EXTS.begin(), IMAGE_EXTS.end(),
                                     [&ext](const char* e){ return ext == e; });
            if (valid) {
                ManifestEntry e;
                e.image_path = p.path();
                entries.push_back(std::move(e));
            }
        }
        std::sort(entries.begin(), entries.end(),
                  [](const ManifestEntry& a, const ManifestEntry& b) {
                      return a.image_path < b.image_path;
                  });
        std::printf("[INFO] Found %zu images in: %s\n",
                    entries.size(), args.image_dir.c_str());
        return entries;
    }

    for (const auto& p : args.image_paths) {
        ManifestEntry e;
        e.image_path = p;
        entries.push_back(std::move(e));
    }
    return entries;
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) try {
    const Args args = parse_args(argc, argv);

    multitask::Config cfg;
    cfg.score_threshold = args.score_thr;
    cfg.nms_threshold   = args.nms_thr;
    cfg.apply_morph     = args.apply_morph;

    std::printf("[INFO] Loading engine: %s\n", args.engine_path.c_str());
    multitask::MultitaskEngine engine(args.engine_path, cfg);
    std::printf("[INFO] Engine ready.\n");

    const std::vector<ManifestEntry> entries = collect_entries(args);
    if (entries.empty())
        throw std::runtime_error(
            "No images to process — use --manifest, --image, or --image_dir");

    fs::create_directories(args.output_dir);

    // ── Per-image inference loop ──────────────────────────────────────────────

    double total_infer_ms = 0.0;
    int    total_dets     = 0;
    int    processed      = 0;

    json results_json = json::array();

    for (const auto& entry : entries) {
        cv::Mat frame = cv::imread(entry.image_path.string());
        if (frame.empty()) {
            std::fprintf(stderr, "[WARN] Cannot read: %s\n",
                         entry.image_path.string().c_str());
            continue;
        }

        const int orig_h = frame.rows;
        const int orig_w = frame.cols;

        const multitask::InferResult result = engine.infer(frame);

        total_infer_ms += result.infer_ms;
        total_dets     += static_cast<int>(result.detections.size());
        ++processed;

        // ── Compose output image ──────────────────────────────────────────────

        cv::Mat display;
        cv::resize(frame, display,
                   cv::Size(multitask::INPUT_W, multitask::INPUT_H),
                   0.0, 0.0, cv::INTER_LINEAR);

        display = overlay_seg(display, result.seg_mask);

        // Ground-truth boxes (scaled from original resolution, drawn first so
        // prediction boxes render on top)
        if (args.show_gt && entry.has_gt) {
            const auto scaled = scale_gt_boxes(entry.gt_boxes, orig_w, orig_h);
            draw_gt_boxes(display, scaled, entry.gt_labels);
        }

        draw_predictions(display, result.detections);

        // Filename watermark
        const std::string stem = entry.image_path.stem().string();
        cv::putText(display, stem, {6, 22},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 0}, 3, cv::LINE_AA);
        cv::putText(display, stem, {6, 22},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, {220, 220, 220}, 1, cv::LINE_AA);

        // ── Save annotated image ──────────────────────────────────────────────

        const fs::path out_path =
            fs::path(args.output_dir) / (stem + "_pred.jpg");
        cv::imwrite(out_path.string(), display,
                    {cv::IMWRITE_JPEG_QUALITY, 92});

        // ── Accumulate JSON result ────────────────────────────────────────────

        if (args.save_json) {
            json jimg;
            jimg["image"]    = entry.image_path.string();
            jimg["infer_ms"] = std::round(result.infer_ms * 10.0) / 10.0;
            jimg["dets"]     = json::array();
            for (const auto& d : result.detections) {
                jimg["dets"].push_back({
                    {"class_id",   d.class_id},
                    {"class_name", d.class_name()},
                    {"score",      std::round(d.score * 1000.0f) / 1000.0f},
                    {"box",        {d.x1, d.y1, d.x2, d.y2}}
                });
            }
            results_json.push_back(jimg);
        }

        std::printf("[%4d/%4zu]  %-40s  dets=%-3zu  GPU=%.1f ms\n",
                    processed,
                    entries.size(),
                    entry.image_path.filename().string().c_str(),
                    result.detections.size(),
                    result.infer_ms);
    }

    // ── Summary ───────────────────────────────────────────────────────────────

    std::printf("\n─── Summary ──────────────────────────────────────────────────\n");
    std::printf("  Images processed : %d / %zu\n", processed, entries.size());
    std::printf("  Total detections : %d  (avg %.1f / image)\n",
                total_dets,
                processed > 0 ? static_cast<double>(total_dets) / processed : 0.0);
    std::printf("  Avg GPU time     : %.1f ms/image\n",
                processed > 0 ? total_infer_ms / processed : 0.0);
    std::printf("  Output dir       : %s\n", args.output_dir.c_str());

    if (args.save_json) {
        const fs::path out_json = fs::path(args.output_dir) / "results.json";
        std::ofstream f(out_json);
        f << results_json.dump(2);
        f.close();
        std::printf("  Results JSON     : %s\n", out_json.string().c_str());
    }

    std::printf("──────────────────────────────────────────────────────────────\n\n");
    return EXIT_SUCCESS;

} catch (const std::exception& ex) {
    std::fprintf(stderr, "[FATAL] %s\n", ex.what());
    return EXIT_FAILURE;
}
