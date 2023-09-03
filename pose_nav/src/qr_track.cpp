#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visp_bridge/image.h>
#include <visp_bridge/poseStamped.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobot.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpServo.h>
#include <visp/vpColVector.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>
#include <tf/transform_datatypes.h>

class AprilTagTracker {
public:
    AprilTagTracker() : r_target(0.2), alpha(0.5), lambda(0.1), k_angular(1.0) {
        // 初始化ROS节点
        ros::NodeHandle nh;

        // 创建ViSP抓取器
        grabber.open("camera/image_raw");
        I.init(480, 640);

        // 设置相机参数（根据您的相机）
        cam_params.initPersProjWithoutDistortion(640, 480, 320, 240);

        // 创建机器人对象
        robot.init(vpRobot::KUKA_KR6_R900);

        // 创建ViSP AprilTag检测器
        tag_detector.setAprilTagFamily(vpAprilTagDetector::TAG_36h11);

        // 订阅机器人关节状态
        js_sub = nh.subscribe("/joint_states", 1, &AprilTagTracker::jointStateCallback, this);

        // 创建速度发布器
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // 初始化标志
        tag_detected = false;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& js_msg) {
        // 获取机器人关节状态
        robot.getJointValues(js_msg, joint_values);
    }

    void run() {
        while (ros::ok()) {
            // 采集图像
            grabber.acquire(I);

            // 检测AprilTag
            tag_detector.detect(I);

            // 如果检测到AprilTag
            if (tag_detector.getNbObjects() > 0) {
                // 获取AprilTag的位姿和方向信息
                vpHomogeneousMatrix cMo = tag_detector.getPose(0, cam_params, r_target);
                vpRotationMatrix R = cMo.getRotationMatrix();

                // 计算机器人当前的朝向角度
                double current_yaw = atan2(R[1][0], R[0][0]);

                // 计算机器人需要调整的角度
                double target_yaw = 0.0; // 设置目标朝向角度
                double yaw_error = target_yaw - current_yaw;

                // 计算机器人的角速度控制命令
                double angular_velocity = k_angular * yaw_error;

                // 发布速度控制命令
                geometry_msgs::Twist cmd_vel_msg;
                cmd_vel_msg.angular.z = angular_velocity;

                cmd_vel_pub.publish(cmd_vel_msg);

                // 设置标志
                tag_detected = true;
            } else {
                tag_detected = false;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    vpImage<unsigned char> I;
    vpCameraParameters cam_params;
    vpAprilTagDetector tag_detector;
    double r_target;
    double alpha;
    double lambda;
    double k_angular;
    ros::Rate rate;
    vpROSGrabber grabber;
    vpROSRobot robot;
    vpColVector joint_values;
    bool tag_detected;
    ros::Subscriber js_sub;
    ros::Publisher cmd_vel_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "april_tag_tracker");
    AprilTagTracker tracker;
    tracker.run();
    return 0;
}

//获取apriltag位姿，对准
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDetectorAprilTag.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRobot.h>
#include <visp/vpServo.h>

int main() {
    // 配置相机参数
    vpCameraParameters cam_params;
    // 设置相机参数，包括焦距、主点等

    // 初始化ViSP
    vpImage<unsigned char> I;
    // 初始化图像采集器或ROS图像消息转换工具

    // 初始化机器人
    vpRobot robot;
    // 配置机器人参数和控制器

    // 设置视觉伺服控制器
    vpServo servo;

    while (true) {
        // 获取相机图像
        // 例如，使用ViSP的图像采集器或ROS图像消息转换工具

        // 检测AprilTag码
        vpDetectorAprilTag detector;
        detector.setCameraParameters(cam_params);
        detector.initTracking();
        if (detector.detect(I)) {
            // 获取AprilTag码的位姿
            vpHomogeneousMatrix cMo = detector.getPose();

            // 计算机器人需要执行的移动操作以对准AprilTag
            // 这可以是基于位置和方向的计算，具体取决于机器人的运动学模型。

            // 在这里执行机器人移动操作，例如，发布速度或位姿控制命令。

            // 设置视觉伺服控制误差
            vpHomogeneousMatrix cMe_desired;  // 期望位姿
            vpHomogeneousMatrix cMe = robot.get_eMec();  // 当前机器人末端位姿
            vpHomogeneousMatrix eMc = cMo.inverse() * cMe;  // 误差位姿

            // 设置伺服控制器参数
            vpAdaptiveGain lambda(0.1, 0.1, 2.0);
            servo.setLambda(lambda);

            // 设置伺服控制器追踪误差
            vpFeatureThetaU feature(eMc);
            servo.setServo(vpServo::EYEINHAND_CAMERA);
            servo.setInteractionMatrixType(vpServo::CURRENT);
            servo.addFeature(feature, feature);
            servo.setCurrentFeature(0);

            // 计算伺服控制命令
            vpColVector q_dot = servo.computeControlLaw();
            // 根据q_dot执行机器人控制，发布速度或位姿控制命令。

        }

        // 循环检测
    }

    return 0;
}

//使用VISP获取ROS仿真摄像头的图像，
// 并识别apriltag码获取对应的位姿数据位姿，
// 并通过这个位姿使机器人和apriltag对齐并且保持20厘米的距离，请给出完整程序

#include <ros/ros.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDetectorAprilTag.h>
#include <visp/vpHomogeneousMatrix.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

vpImage<unsigned char> I; // ViSP图像
vpCameraParameters cam_params; // 相机参数
ros::Publisher cmd_vel_pub; // 发布机器人速度控制命令的ROS话题

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // 将ROS图像消息转换为ViSP图像
    visp_bridge::toVispImage(*msg, I);
}

void apriltagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    // 获取AprilTag码的位姿
    tf::Quaternion q(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z, pose_msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 计算机器人需要执行的移动操作以对准AprilTag并保持20厘米距离
    // 这可以是基于位置和方向的计算，具体取决于机器人的运动学模型。

    // 在这里执行机器人移动操作，例如发布速度控制命令。
    geometry_msgs::Twist cmd_vel_msg;
    
    // 计算到达目标距离的线速度
    double target_distance = 0.20; // 20厘米
    double current_distance = 0.0; // 从位姿数据中获取当前距离
    double linear_speed = 0.1; // 适当的线速度增益
    double linear_error = target_distance - current_distance;
    cmd_vel_msg.linear.x = linear_speed * linear_error;

    // 计算对准目标方向的角速度
    double target_yaw = 0.0; // 目标角度
    double angular_speed = 0.1; // 适当的角速度增益
    double angular_error = target_yaw - yaw;
    cmd_vel_msg.angular.z = angular_speed * angular_error;

    cmd_vel_pub.publish(cmd_vel_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_alignment");
    ros::NodeHandle nh;

    // 初始化ViSP相机参数
    cam_params.initPersProjWithoutDistortion(640, 480, 320, 240);

    // 创建ViSP AprilTag检测器
    vpDetectorAprilTag detector;
    detector.setCameraParameters(cam_params);
    detector.initAprilTag36h11();

    // 订阅相机图像话题
    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 1, imageCallback);

    // 订阅AprilTag码位姿话题
    ros::Subscriber pose_sub = nh.subscribe("/apriltag/pose", 1, apriltagPoseCallback);

    // 创建发布机器人速度控制命令的ROS话题
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // 主循环
    ros::Rate rate(30); // 30Hz
    while (ros::ok()) {
        // 检测AprilTag码
        detector.detect(I);

        // 在这里可以进一步处理ViSP图像，或者执行其他操作

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobot.h>
#include <visp_ros/vpROSPlugin.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpAdaptiveGain.h>

// 全局变量
vpImage<unsigned char> I; // ViSP图像
vpCameraParameters cam_params; // 相机参数
ros::Publisher cmd_vel_pub; // 发布机器人速度控制命令的ROS话题
geometry_msgs::PoseStamped apriltag_pose; // 存储AprilTag位姿

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // 将ROS图像消息转换为ViSP图像
    visp_bridge::toVispImage(*msg, I);
}

void apriltagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    // 存储AprilTag位姿数据
    apriltag_pose = *pose_msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_servo");
    ros::NodeHandle nh;

    // 初始化ViSP相机参数
    cam_params.initPersProjWithoutDistortion(640, 480, 320, 240);

    // 创建ViSP图像采集器
    vpROSGrabber grabber;
    grabber.open("ros://camera/image_raw"); // 替换为您的摄像头话题

    // 创建ViSP机器人对象
    vpROSRobot robot;
    robot.setBaseFrame("base_link"); // 机器人基坐标系
    robot.setCameraParameters(cam_params);

    // 创建视觉伺服控制器
    vpServo servo;
    servo.setCameraParameters(cam_params);
    servo.setServo(vpServo::EYEINHAND_CAMERA);
    servo.setInteractionMatrixType(vpServo::CURRENT);
    servo.setLambda(0.1);

    // 创建特征点
    vpFeatureThetaU feature;
    feature.buildFrom(apriltag_pose.pose);

    // 设置特征点到目标的误差
    vpFeaturePoint p_error;
    p_error.setWorldCoordinates(0.2, 0.0, 0.0); // 20厘米的距离
    vpFeaturePoint pd;
    pd.buildFrom(apriltag_pose.pose);
    p_error.track(pd);

    // 将特征点添加到伺服控制器
    servo.addFeature(feature, p_error);

    // 创建ROS速度控制话题
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // 订阅相机图像话题
    ros::Subscriber image_sub = nh.subscribe("camera/image_raw", 1, imageCallback);

    // 订阅AprilTag码位姿话题
    ros::Subscriber pose_sub = nh.subscribe("/apriltag/pose", 1, apriltagPoseCallback);

    // 主循环
    ros::Rate rate(30); // 30Hz
    while (ros::ok()) {
        // 获取摄像头图像
        grabber.acquire(I);

        // 计算伺服控制命令
        vpColVector q_dot = servo.computeControlLaw();
        
        // 发布速度控制命令
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = q_dot[0];
        cmd_vel_msg.angular.z = q_dot[1];
        cmd_vel_pub.publish(cmd_vel_msg);

        // 循环处理
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
