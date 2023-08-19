#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <iostream>

class PolygonPathPlanner
{
public:
    PolygonPathPlanner() : nh("~")
    {
        // 订阅多边形区域信息
        polygon_sub = nh.subscribe("polygon_area", 1, &PolygonPathPlanner::polygonCallback, this);

        // 发布路径
        path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);

        // 获取坐标转换
        tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
    }

    void polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg)
    {
        // 在这里执行路径规划逻辑，计算从机器人当前位置到多边形区域内的路径

        // 示例：将多边形的中心作为路径的起点
        geometry_msgs::PoseStamped start_pose;
        start_pose.header = polygon_msg->header;
        start_pose.pose.position.x = (polygon_msg->polygon.points[0].x + polygon_msg->polygon.points[2].x) / 2.0;
        start_pose.pose.position.y = (polygon_msg->polygon.points[0].y + polygon_msg->polygon.points[2].y) / 2.0;
        start_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

        // 示例：将机器人当前位置作为路径的终点
        geometry_msgs::PoseStamped end_pose;
        tf_listener.transformPose("map", ros::Time(0), geometry_msgs::PoseStamped(), "base_link", end_pose);

        // 生成路径消息
        nav_msgs::Path path;
        path.header = start_pose.header;
        path.poses.push_back(start_pose);
        path.poses.push_back(end_pose);

        // 发布路径
        path_pub.publish(path);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber polygon_sub;
    ros::Publisher path_pub;
    tf::TransformListener tf_listener;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "polygon_path_planner");
    PolygonPathPlanner planner;

    ros::spin();

    return 0;
}
