// ROS 2 C++ driver for the Insta360 X4 / X5 in USB/UVC webcam mode.
//
// Discovery: scans /sys/class/video4linux/ and matches by USB vendor ID
// (0x2e1a) or a case-insensitive "insta360" name substring — never assumes
// a fixed /dev/videoN path (port numbers change on replug, see CLAUDE.md Rule 1).
//
// Reconnects automatically after MAX_CONSECUTIVE_FAILURES missed frames.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace fs = std::filesystem;

static constexpr const char* INSTA360_VID         = "2e1a";
static constexpr int         MAX_CONSEC_FAILURES   = 15;

// ── helpers ───────────────────────────────────────────────────────────────────

static std::string read_sysfs(const fs::path& p)
{
    std::ifstream f(p);
    if (!f.is_open()) return {};
    return {std::istreambuf_iterator<char>(f), {}};
}

static std::string to_lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

// Returns "/dev/videoN" for the first Insta360 UVC device found, or "".
static std::string find_insta360()
{
    const fs::path root = "/sys/class/video4linux";
    std::error_code ec;
    if (!fs::exists(root, ec)) return {};

    std::vector<fs::path> entries;
    for (auto& e : fs::directory_iterator(root, ec))
        entries.push_back(e.path());
    std::sort(entries.begin(), entries.end());

    for (const auto& dir : entries) {
        // Match by USB vendor ID in the uevent file
        if (to_lower(read_sysfs(dir / "device" / "uevent")).find(INSTA360_VID)
                != std::string::npos)
            return "/dev/" + dir.filename().string();

        // Fallback: v4l2 device name substring
        if (to_lower(read_sysfs(dir / "name")).find("insta360")
                != std::string::npos)
            return "/dev/" + dir.filename().string();
    }
    return {};
}

// ── node ─────────────────────────────────────────────────────────────────────

class Insta360Node : public rclcpp::Node
{
public:
    Insta360Node() : Node("insta360_node")
    {
        declare_parameter("device",   std::string(""));   // "" = auto-discover
        declare_parameter("frame_id", std::string("insta360_optical_frame"));
        declare_parameter("width",    2880);
        declare_parameter("height",   1440);
        declare_parameter("fps",      30.0);

        explicit_device_ = get_parameter("device").as_string();
        frame_id_        = get_parameter("frame_id").as_string();
        width_           = get_parameter("width").as_int();
        height_          = get_parameter("height").as_int();
        fps_             = get_parameter("fps").as_double();

        pub_ = create_publisher<sensor_msgs::msg::Image>(
            "camera/image_raw", rclcpp::SensorDataQoS());

        if (!open_device())
            RCLCPP_WARN(get_logger(), "Insta360 not found at startup — will retry");

        const double period_s = 1.0 / std::max(fps_, 1.0);
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period_s),
            [this]() { tick(); });
    }

private:
    // ── device management ─────────────────────────────────────────────────────

    std::string resolve_device()
    {
        if (!explicit_device_.empty()) return explicit_device_;

        const std::string dev = find_insta360();
        if (dev.empty())
            RCLCPP_ERROR(get_logger(),
                "No Insta360 UVC device found — check webcam mode is enabled "
                "and the USB-C cable is seated");
        return dev;
    }

    bool open_device()
    {
        const std::string dev = resolve_device();
        if (dev.empty()) return false;

        auto cap = std::make_unique<cv::VideoCapture>(dev, cv::CAP_V4L2);
        if (!cap->isOpened()) {
            RCLCPP_ERROR(get_logger(), "VideoCapture failed to open %s", dev.c_str());
            return false;
        }

        // Force MJPEG before setting resolution/fps — YUYV (the default) is
        // limited to ~10 fps at 1920×1080 by USB bandwidth; MJPEG lifts that.
        cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap->set(cv::CAP_PROP_FRAME_WIDTH,  static_cast<double>(width_));
        cap->set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
        cap->set(cv::CAP_PROP_FPS,          fps_);

        cap_      = std::move(cap);
        failures_ = 0;

        RCLCPP_INFO(get_logger(),
            "Streaming %s @ %dx%d / %.0f fps", dev.c_str(), width_, height_, fps_);
        return true;
    }

    void release_device()
    {
        cap_.reset();
    }

    // ── timer callback ────────────────────────────────────────────────────────

    void tick()
    {
        if (!cap_) {
            open_device();
            return;
        }

        cv::Mat frame;
        if (!cap_->read(frame) || frame.empty()) {
            if (++failures_ >= MAX_CONSEC_FAILURES) {
                RCLCPP_WARN(get_logger(),
                    "%d consecutive frame failures — releasing handle and re-discovering",
                    MAX_CONSEC_FAILURES);
                release_device();
            }
            return;
        }

        failures_ = 0;

        std_msgs::msg::Header hdr;
        hdr.stamp    = now();
        hdr.frame_id = frame_id_;

        pub_->publish(*cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg());
    }

    // ── members ───────────────────────────────────────────────────────────────

    std::string explicit_device_;
    std::string frame_id_;
    int         width_{1920};
    int         height_{1080};
    double      fps_{30.0};

    std::unique_ptr<cv::VideoCapture>                     cap_;
    int                                                   failures_{0};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                          timer_;
};

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Insta360Node>());
    rclcpp::shutdown();
}
