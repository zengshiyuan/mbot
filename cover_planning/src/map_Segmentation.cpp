#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <iostream>
#include <string>


// #include <dubins_path_planner/dubins_path.h>

// 将多边形区域转换为网格： 将选定的多边形区域映射到地图网格上，将区域内的网格单元标记为需要覆盖的区域。

// 在网格区域内进行地图分割： 在标记为需要覆盖的网格区域内，进行地图分割，生成可覆盖的路径规划点。

// 执行路径规划： 对生成的路径规划点执行路径规划，确定机器人在区域内的移动路径。

// 以下是一个示例程序，展示了如何将选定的多边形区域转换为网格，并在网格区域内进行地图分割：

class map_Segmentation
{
private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Subscriber polygon_sub;
    ros::Publisher path_pub;
    ros::Publisher grid_pub;

    int map_width,map_height;
    double map_resolution;

    geometry_msgs::Polygon select_polygon;
    nav_msgs::OccupancyGrid grid_map;
public:
    map_Segmentation();
    bool isInsidePolygon(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon);
    void coverPolygontoGrid(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
    void segmentMAP(const nav_msgs::OccupancyGrid::ConstPtr &map_msgs);
    void polygonCB(const geometry_msgs::PolygonStamped::ConstPtr &polygon_msg);
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
};

map_Segmentation::map_Segmentation():nh("~")
{
    map_sub = nh.subscribe("map",1,&map_Segmentation::mapCB,this);
    polygon_sub = nh.subscribe("polygon_map",1,&map_Segmentation::polygonCB,this);
    path_pub = nh.advertise<nav_msgs::Path>("cover_path",1);
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
}
void map_Segmentation::mapCB(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    map_width = map_msg->info.width;
    map_height = map_msg->info.height;
    map_resolution = map_msg->info.resolution;

    //在地图初始化后进行地图分割
    if (!select_polygon.points.empty())
    {
        coverPolygontoGrid(map_msg);
        // segmentMAP(map_msg);
    }
    

}
void map_Segmentation::polygonCB(const geometry_msgs::PolygonStamped::ConstPtr &polygon_msg)
{
    select_polygon = polygon_msg->polygon;
}
void map_Segmentation::segmentMAP(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    //在网格区域内进行地图分割
    nav_msgs::Path cover_path;
    for (int x = 0; x < map_width; ++x)
    {
        for (int y = 0; y < map_height; ++y)
        {
            if (grid_map.data[y*map_width+x]==100)   //需要覆盖的区域    
            {
                //将网格中的覆盖区域映射到实际坐标中
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";
                pose.pose.position.x = x * map_resolution;
                pose.pose.position.y = y * map_resolution;
                cover_path.poses.push_back(pose);
            }
            
        }
        
    }
    std::cout<<"发布path"<<std::endl;
    // 执行弓形路径规划
    // DubinsPath dubins_path;
    // bool path_found = dubins_path.planPath(cover_path, 0.2);  // 0.2为机器人的转向半径，根据实际情况调整
    //发布覆盖路径
    path_pub.publish(cover_path);
    
}

void map_Segmentation::coverPolygontoGrid(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    //初始化网格地图
    grid_map.info = map_msg->info;
    grid_map.data.resize(map_width,map_height);

    for (int x = 0; x < map_width; ++x)
    {
        for (int y = 0; y < map_height; ++y)
        {
            geometry_msgs::Point point;
            point.x = x*map_resolution;
            point.y = y*map_resolution;

            if (isInsidePolygon(point,select_polygon))
            {
                //将多边形内的网格单元标记为需要覆盖的区域
                grid_map.data[y*map_width+x] = 100; //100表表示需要覆盖的区域
            }
            else
            {
                grid_map.data[y*map_width+x] = -1;  //-1表示不需要覆盖的区域
            }
        }
    }
    grid_pub.publish(grid_map);
}


bool map_Segmentation::isInsidePolygon(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon)
{
    // 简单地检查点是否在多边形区域内（可根据需求调整）
    // 这里省略了实际的多边形内部判断算法
    // 可以使用射线法等方法判断点是否在多边形内部

    int num_vertices = polygon.points.size();
    if (num_vertices)
    {
        return false;   //无法构成多边形
    }
    int intersections = 0;
    for (int i = 0,j=num_vertices-1; i < num_vertices; j=i++)
    {
        if (((polygon.points[i].y>point.y)!=(polygon.points[j].y>point.y))&&
            (point.x < (polygon.points[j].x - polygon.points[i].x) * (point.y - polygon.points[i].y) /
                            (polygon.points[j].y - polygon.points[i].y) +
                        polygon.points[i].x))
        {
            intersections++;
        }
    }
    
    

    // 这里返回一个简单的示例结果
    return (intersections %2) == 1;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"map_segmentation_node");
    map_Segmentation map_seg;
    ros::spin();
}
