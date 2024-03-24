#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

std::string path_output = "/home/umelab-pgi5g/softhand_data/";

class DistanceMeasurementNode {
public:
    DistanceMeasurementNode() {
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        color_sub_ = it.subscribe("/camera/color/image_raw", 1, &DistanceMeasurementNode::colorImageCallback, this);
        depth_sub_ = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &DistanceMeasurementNode::depthImageCallback, this);
        image_pub_ = it.advertise("/kai_processed_image", 1);
    }

    void colorImageCallback(const sensor_msgs::ImageConstPtr& color_msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
            color_image_ = cv_ptr->image;
            
            cv::imshow("Color Image", color_image_);
            int key = cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
        if (color_image_.empty()) {
            ROS_WARN("No color image available");
            return;
        }

        try {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_image_ = cv_ptr->image;
            processImage();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void processImage() {
    // color
    cv::Mat color_th = color_image_.clone();

    cv::Mat hsv_image;
    cv::cvtColor(color_th, hsv_image, cv::COLOR_BGR2HSV);

    // HSVのしきい値を設定
    cv::Scalar lower_thresh(0, 100, 100); // 下限値 (H:0, S:64, V:0)
    cv::Scalar upper_thresh(10, 255, 255); // 上限値 (H:30, S:255, V:255)

    // HSV画像から指定された色の物体を抽出するマスクを作成
    cv::Mat mask;
    cv::inRange(hsv_image, lower_thresh, upper_thresh, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2);

    // 物体の輪郭を見つけるための変数
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // 物体の輪郭を見つける
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 最大面積を見つける
    double max_area = 0;
    int max_contour_index = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_contour_index = i;
        }
    }

    // 最大面積の物体が見つかった場合
    if (max_contour_index != -1) {
        // 最大面積の物体の輪郭を描画するマスクを作成
        cv::Mat max_area_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        cv::drawContours(max_area_mask, contours, max_contour_index, cv::Scalar(255), cv::FILLED);

        // 最大面積の領域以外のピクセルを白色に塗りつぶす
        cv::Mat white_mask = cv::Mat::ones(color_th.size(), CV_8UC1) * 255;
        max_area_mask = 255 - max_area_mask; // 最大面積の領域以外をマスク
        color_th.setTo(cv::Scalar(255, 255, 255), max_area_mask);
    } else {
        // 物体が見つからない場合、全体を白色にする
        color_th.setTo(cv::Scalar(255, 255, 255));
    }

    // ---------------
        // distance
        cv::Mat processed_image = color_image_.clone();

        for (int y = 0; y < depth_image_.rows; y++) {
            for (int x = 0; x < depth_image_.cols; x++) {
                uint16_t depth_value = depth_image_.at<uint16_t>(cv::Point(x, y));
                float distance = depth_value * 0.001f; // Convert mm to meters

                if (distance > max_distance_ || distance == 0) {
                    processed_image.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(255, 255, 255); // Set to white
                }
            }
        }
    
    // ------------
    // Combine the images
    for (int y = 0; y < color_th.rows; ++y) {
        for (int x = 0; x < color_th.cols; ++x) {
            // Check if either pixel is not white
            if (color_th.at<cv::Vec3b>(y, x) != cv::Vec3b(255, 255, 255)) 
            {
                // If not white, copy from processed_image
                processed_image.at<cv::Vec3b>(y, x) = color_th.at<cv::Vec3b>(y, x);
            }
        }
    }

    // Display the processed image
    cv::imshow("Processed Image", processed_image);
    int key = cv::waitKey(1);
    if (key == 'n')
        imwrite(path_output + "img_process_.jpg", processed_image);
    else if (key == 'c')
        imwrite(path_output + "img_color.jpg", color_image_);
    else if (key == 'q')
        return;

    // Publish processed image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", processed_image).toImageMsg();
    image_pub_.publish(msg);
    }

private:
    image_transport::Subscriber color_sub_;
    ros::Subscriber depth_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat color_image_;
    cv::Mat depth_image_;
    float max_distance_ = 0.12; // Maximum distance to display, in meters   # 11
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_measurement_node");
    DistanceMeasurementNode node;
    ros::spin();
    return 0;
}
