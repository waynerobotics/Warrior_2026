// seg_viz_node.cpp
//
// Overlays the lane segmentation mask on the raw camera image.
//
// Subscribes:
//   camera/image_raw   (sensor_msgs/Image, bgr8)
//   ai/seg_mask        (sensor_msgs/Image, mono8  — values 0/1)
// Publishes:
//   ai/seg_viz         (sensor_msgs/Image, bgr8)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <mutex>

class SegVizNode : public rclcpp::Node
{
public:
    SegVizNode() : Node("seg_viz_node")
    {
        auto qos = rclcpp::SensorDataQoS().keep_last(1);

        sub_mask_ = create_subscription<sensor_msgs::msg::Image>(
            "ai/seg_mask", qos,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mask_mutex_);
                latest_mask_ = msg;
            });

        sub_img_ = create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", qos,
            std::bind(&SegVizNode::on_image, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::Image>("ai/seg_viz", 10);

        RCLCPP_INFO(get_logger(), "seg_viz_node ready → ai/seg_viz");
    }

private:
    void on_image(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        sensor_msgs::msg::Image::SharedPtr mask_msg;
        {
            std::lock_guard<std::mutex> lk(mask_mutex_);
            mask_msg = latest_mask_;
        }

        cv_bridge::CvImageConstPtr cv_img;
        try {
            cv_img = cv_bridge::toCvShare(img_msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge img: %s", e.what());
            return;
        }

        // No mask yet — publish the raw frame so the topic is always alive
        if (!mask_msg) {
            pub_->publish(*img_msg);
            return;
        }

        cv_bridge::CvImageConstPtr cv_mask;
        try {
            cv_mask = cv_bridge::toCvShare(mask_msg, "mono8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge mask: %s", e.what());
            return;
        }

        cv::Mat mask_resized;
        cv::resize(cv_mask->image, mask_resized,
                   cv_img->image.size(), 0, 0, cv::INTER_NEAREST);

        cv::Mat overlay = cv_img->image.clone();
        overlay.setTo(cv::Scalar(0, 200, 0), mask_resized > 0);

        cv::Mat blended;
        cv::addWeighted(cv_img->image, 0.70, overlay, 0.30, 0.0, blended);

        pub_->publish(*cv_bridge::CvImage(img_msg->header, "bgr8", blended).toImageMsg());
    }

    std::mutex mask_mutex_;
    sensor_msgs::msg::Image::SharedPtr latest_mask_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mask_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SegVizNode>());
    rclcpp::shutdown();
    return 0;
}
