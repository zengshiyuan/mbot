#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

ros::Publisher cmd_vel_pub;
image_transport::Subscriber image_sub;
bool tag_detected = false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (!tag_detected) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // 在这里，您可以使用OpenCV进行图像处理和AprilTag的检测
        // 请确保在此处检测到AprilTag并设置tag_detected为true
        // 示例代码：使用AprilTag库检测AprilTag
        if (detectAprilTag(cv_ptr->image)) {
            tag_detected = true;
        }
    }
}

void aprilTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    if (msg->detections.size() > 0) {
        // 如果检测到AprilTag，则将机器人对准AprilTag中心
        tag_detected = true;

        // 获取AprilTag的位置信息
        double tag_x = msg->detections[0].pose.pose.pose.position.x;
        double tag_y = msg->detections[0].pose.pose.pose.position.y;

        // 创建Twist消息来控制机器人运动
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.1; // 设置线速度
        cmd_vel.angular.z = -0.01 * tag_x; // 根据X轴偏差进行旋转控制

        // 发布控制命令
        cmd_vel_pub.publish(cmd_vel);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "apriltag_tracker");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    image_transport::ImageTransport it(nh);
    image_sub = it.subscribe("/camera/image_raw", 1, imageCallback);

    ros::Subscriber april_tag_sub = nh.subscribe("/tag_detections", 1, aprilTagCallback);

    ros::spin();

    return 0;
}
