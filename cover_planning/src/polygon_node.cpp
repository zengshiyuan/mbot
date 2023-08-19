#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#include <iostream>
struct polygon_point
{
public:
    float x;
    float y;

    polygon_point()
    {
        x = 0.0;
        y = 0.0;
    };
    polygon_point(const float xx,const float yy):x(xx),y(yy)
    {};
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"polygon_node");
    ros::NodeHandle nh;
    ros::Publisher polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("polygon_map",1); 

    geometry_msgs::Polygon polygon;
    geometry_msgs::PolygonStamped myPolygon;
    geometry_msgs::Point32 p1,p2,p3,p4,p5,p6,p7,p8;


    p1.x=3.36;
    p1.y=0;
    p2.x=3.36;
    p2.y=-2.0;
    p3.x=0.7;
    p3.y=-2.0;
    p4.x=0.7;
    p4.y=-4.8;
    p5.x=-2;
    p5.y=-4.8;
    p6.x=-1.9;
    p6.y=4.0;
    p7.x=-0.55;
    p7.y=4.0;
    p8.x=-0.55;
    p8.y=0;

    myPolygon.header.frame_id = "map";
    myPolygon.polygon.points.push_back(p1);
    myPolygon.polygon.points.push_back(p2);
    myPolygon.polygon.points.push_back(p3);
    myPolygon.polygon.points.push_back(p4);
    myPolygon.polygon.points.push_back(p5);
    myPolygon.polygon.points.push_back(p6);
    myPolygon.polygon.points.push_back(p7);
    myPolygon.polygon.points.push_back(p8);
    ros::Rate rate(10);
    while (ros::ok())
    {
        myPolygon.header.stamp = ros::Time::now();
        polygon_pub.publish(myPolygon);
        rate.sleep();
    }
    return 0;
}