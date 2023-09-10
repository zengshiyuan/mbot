#include <ros/ros.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobot.h>
#include <visp/vpImage.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpDetectorAprilTag.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "visp_visual_servo");
    ros::NodeHandle nh;

    // 创建ROS图像采集器（虚拟相机）
    vpROSGrabber g;
    g.setCameraInfoTopic("/camera/camera_info");
    g.setImageTopic("/camera/image_raw");
    g.open(argc, argv);

    // 创建ROS机器人接口（虚拟机器人）
    vpROSRobot robot;
    robot.initRos("/cmd_vel");

    // 初始化相机参数（虚拟相机参数，根据实际情况修改）
    vpCameraParameters cam_params;
    cam_params.initPersProjWithoutDistortion(400, 400, 320, 240);

    // 创建视觉伺服任务
    vpServo task;
    vpFeaturePoint point; // 2D特征点
    vpFeatureThetaU theta_u; // 姿态伺服特征

    // 设置视觉伺服参数
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT,vpServo::RECURSIVE ,vpServo::PSEUDO_INVERSE);

    // 创建AprilTag检测器
    vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);

    // 设置AprilTag检测器参数
    detector.setAprilTagQuadDecimate(1.0); // 设置检测器的减采样因子
    double tagSize = 0.1; // AprilTag的真实尺寸（以米为单位）
    detector.setAprilTagSize(tagSize);

    // 设置机器人和AprilTag的期望距离
    double desired_distance = 0.3;

    // 主循环
    while (ros::ok()) {
        // 采集图像
        if (g.acquire()) {
            vpImage<unsigned char> I;
            // g.getImage(I);

            // 检测AprilTag
            detector.detect(I, cam_params);

            if (detector.getNbObjects() > 0) {
                // 获取第一个检测到的AprilTag的位姿信息
                vpHomogeneousMatrix cMo = detector.getPose(0);

                // 计算2D特征点坐标
                point.set_x(cam_params.get_px() * cMo[0][3] / cMo[2][3] + cam_params.get_u0());
                point.set_y(cam_params.get_py() * cMo[1][3] / cMo[2][3] + cam_params.get_v0());
                task.addFeature(point);

                // 计算姿态伺服特征
                vpMatrix R = cMo.getRotationMatrix();
                vpThetaUVector tu;
                tu.buildFrom(R);
                theta_u.buildFrom(tu);
                task.addFeature(theta_u);

                // 计算视觉伺服的控制命令
                vpColVector v = task.computeControlLaw();

                // 计算机器人和AprilTag的距离
                double current_distance = cMo[2][3];

                // 计算机器人前进速度，以保持期望距离
                double desired_velocity = (current_distance - desired_distance) * 0.1;

                // 控制机器人运动（虚拟机器人运动，根据实际情况修改）
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = desired_velocity; // X方向线速度
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = v[2]; // 角速度
                // robot.setTwist(cmd_vel);
            }
        }
    }
}
       

//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpAprilTagDetector.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobot.h>
#include <visp_ros/vpROSRobotVisualServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpCameraParameters.h>

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "visp_apriltag_tracking");
    ros::NodeHandle nh;
    
    // 创建ROS图像订阅器
    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 1, imageCallback);
    
    // 创建ROS机器人控制发布器
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    // 创建Visp相机参数（根据实际相机配置进行设置）
    vpCameraParameters cam_params;
    cam_params.initPersProjWithoutDistortion(640, 480, 320, 240);
    
    // 创建Visp相机
    vpROSGrabber g;
    g.setCameraParameters(cam_params);
    
    // 创建Visp AprilTag检测器
    vpAprilTagDetector detector(vpAprilTagDetector::TAG_36h11);
    detector.setAprilTagQuadDecimate(2.0);
    detector.setAprilTagPoseEstimationMethod(vpAprilTagDetector::HOMOGRAPHY_VIRTUAL_VS);
    
    // 创建Visp机器人控制接口
    vpROSRobot robot;
    robot.setCmdVelTopic("/cmd_vel");
    
    // 创建Visp视觉伺服任务
    vpROSRobotVisualServo vs;
    
    // 创建2D特征点特征（用于控制AprilTag码在图像中心的位置）
    vpFeaturePoint s;
    s.setInteractionMatrixType(vpFeaturePoint::DESIRED);
    s.setCameraParameters(cam_params);
    s.set_xy(0.0, 0.0); // 设置特征点在图像中心的位置
    
    // 创建姿态伺服特征（用于机器人与AprilTag码的朝向一致）
    vpFeatureThetaU thetaU;
    thetaU.setInteractionMatrixType(vpFeatureThetaU::DESIRED);
    thetaU.setThetaU(0.0, 0.0, 0.0); // 设置期望的姿态
    
    // 添加特征到视觉伺服任务
    vs.addFeature(s);
    vs.addFeature(thetaU);
    
    // 主循环
    while (ros::ok()) {
        // 从相机获取图像
        vpImage<unsigned char> I;
        g.getImage(I);
        
        // 检测AprilTag
        std::vector<vpHomogeneousMatrix> cMo_vec;
        if (detector.detect(I, cam_params, cMo_vec) > 0) {
            // 获取AprilTag的姿态信息
            vpHomogeneousMatrix cMo = cMo_vec[0];
            
            // 更新2D特征点的位置，将AprilTag保持在图像中心的中垂线上
            double tag_x = cMo[0][3] / cMo[2][3]; // AprilTag在图像中心的x坐标
            double tag_y = cMo[1][3] / cMo[2][3]; // AprilTag在图像中心的y坐标
            s.set_xy(tag_x, tag_y);
            
            // 更新姿态伺服特征的姿态信息，可以根据需要设置朝向一致的期望姿态
            // 在这个示例中，不对姿态信息进行更新
            
            // 计算控制命令
            vs.computeControlLaw();
            
            // 获取机器人的线速度和角速度
            vpColVector v = vs.getRobotVelocity();
            
            // 发布控制命令到机器人
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = v[0];
            cmd_vel.linear.y = v[1];
            cmd_vel.angular.z = v[2];
            cmd_vel_pub.publish(cmd_vel);
        }
        
        ros::spinOnce();
    }
    
    return 0;
}



// 3D
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint3D.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpRobotAfma6.h>
#include <iostream>

int main() {
    // 创建视觉伺服任务
    vpServo task;
    
    // 创建机器人模型（示例中使用vpRobotAfma6，请根据您的机器人类型进行调整）
    vpRobotAfma6 robot;
    
    // 创建3D特征点特征
    vpFeaturePoint3D s;
    s.setInteractionMatrixType(vpFeaturePoint3D::DESIRED);
    s.buildFrom(0.0, 0.0, 0.0); // 设置期望的3D特征点位置
    
    // 创建姿态伺服特征
    vpFeatureThetaU thetaU;
    thetaU.setInteractionMatrixType(vpFeatureThetaU::DESIRED);
    thetaU.setThetaU(0.0, 0.0, 0.0); // 设置期望的姿态
    
    // 添加特征到伺服任务
    task.addFeature(s);
    task.addFeature(thetaU);
    
    // 主循环
    while (true) {
        // 计算控制命令
        vpColVector v = task.computeControlLaw();
        
        // 将控制命令应用到机器人
        robot.setVelocity(vpRobotAfma6::TRANS_VYR, v[0], v[1], v[2]);
        
        // 模拟视觉测量更新（假设视觉特征在每次循环中更新）
        // 通常，您需要与视觉传感器集成，获取实际的视觉测量值
        s.buildFrom(0.0, 0.0, 0.0); // 更新3D特征点位置
        thetaU.setThetaU(0.0, 0.0, 0.0); // 更新姿态伺服特征
        
        // 模拟循环延时
        vpTime::wait(10000); // 等待10毫秒
    }
    
    return 0;
}


