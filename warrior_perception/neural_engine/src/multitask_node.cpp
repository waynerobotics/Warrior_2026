// multitask_node.cpp
//
// ROS 2 node: subscribes camera/image_raw, runs TRT inference, publishes
// ai/seg_mask (sensor_msgs/Image, mono8) and ai/detections (Detection2DArray).

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cuda_runtime.h>

#include "multitask_engine.hpp"

class MultitaskNode : public rclcpp::Node
{
public:
    MultitaskNode() : Node("multitask_node")
    {
        declare_parameter("engine_path",         "");
        declare_parameter("image_topic",         "camera/image_raw");
        declare_parameter("seg_topic",           "ai/seg_mask");
        declare_parameter("detections_topic",    "ai/detections");
        declare_parameter("score_threshold",     0.20);
        declare_parameter("nms_threshold",       0.50);
        declare_parameter("apply_morph",         false);

        const auto engine_path      = get_parameter("engine_path").as_string();
        const auto image_topic      = get_parameter("image_topic").as_string();
        const auto seg_topic        = get_parameter("seg_topic").as_string();
        const auto detections_topic = get_parameter("detections_topic").as_string();

        if (engine_path.empty())
            throw std::runtime_error("engine_path parameter must be set");

        multitask::Config cfg;
        cfg.score_threshold = static_cast<float>(get_parameter("score_threshold").as_double());
        cfg.nms_threshold   = static_cast<float>(get_parameter("nms_threshold").as_double());
        cfg.apply_morph     = get_parameter("apply_morph").as_bool();

        engine_ = std::make_unique<multitask::MultitaskEngine>(engine_path, cfg);

        // Depth-1 sensor QoS: always process the latest frame, never queue old ones.
        auto qos = rclcpp::SensorDataQoS().keep_last(1);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic, qos,
            std::bind(&MultitaskNode::on_image, this, std::placeholders::_1));

        pub_seg_ = create_publisher<sensor_msgs::msg::Image>(seg_topic, 10);
        pub_det_ = create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic, 10);

        cudaFree(0);  // force CUDA context init before first inference

        RCLCPP_INFO(get_logger(),
            "multitask_node ready (engine=%s, img=%s, seg=%s, det=%s)",
            engine_path.c_str(), image_topic.c_str(),
            seg_topic.c_str(), detections_topic.c_str());
    }

private:
    void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_img;
        try {
            cv_img = cv_bridge::toCvShare(msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
            return;
        }
        multitask::InferResult result;
        try {
            result = engine_->infer(cv_img->image);
        result.seg_mask.convertTo(result.seg_mask, CV_8UC1, 255.0);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "inference: %s", e.what());
            return;
        }

        // ── Segmentation mask ─────────────────────────────────────────────────
        auto seg_msg = cv_bridge::CvImage(msg->header, "mono8", result.seg_mask).toImageMsg();
        pub_seg_->publish(*seg_msg);

        // ── Detections ────────────────────────────────────────────────────────
        vision_msgs::msg::Detection2DArray det_arr;
        det_arr.header = msg->header;

        for (const auto& d : result.detections) {
            vision_msgs::msg::Detection2D det;
            det.header          = msg->header;
            det.bbox.center.position.x = static_cast<double>(d.cx());
            det.bbox.center.position.y = static_cast<double>(d.cy());
            det.bbox.size_x     = static_cast<double>(d.x2 - d.x1);
            det.bbox.size_y     = static_cast<double>(d.y2 - d.y1);

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = d.class_name();
            hyp.hypothesis.score    = static_cast<double>(d.score);
            det.results.push_back(hyp);

            det_arr.detections.push_back(det);
        }

        pub_det_->publish(det_arr);

        RCLCPP_DEBUG(get_logger(),
            "%zu detections  infer=%.1f ms  total=%.1f ms",
            result.detections.size(), result.infer_ms, result.total_ms);
    }

    std::unique_ptr<multitask::MultitaskEngine>                       engine_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr             pub_seg_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr  pub_det_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultitaskNode>());
    rclcpp::shutdown();
    return 0;
}
