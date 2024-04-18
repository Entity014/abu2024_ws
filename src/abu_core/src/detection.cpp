#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class Detection : public rclcpp::Node
{
public:
    Detection()
        : Node("detection_node")
    {
        // Subscribe to the image topic
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/color/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&Detection::sub_image_callback, this, std::placeholders::_1));
    }

private:
    void sub_image_callback(const sensor_msgs::msg::Image &image_data)
    {
        try
        {
            // Convert ROS Image message to OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(image_data, sensor_msgs::image_encodings::BGR8);

            // Display the received image
            cv::imshow("Received Image", cv_ptr->image);
            cv::waitKey(1); // Adjust wait time as needed
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Detection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
