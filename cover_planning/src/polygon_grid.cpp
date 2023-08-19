#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>

class PolygonToGridConverter
{
public:
    PolygonToGridConverter() : nh("~")
    {
        map_sub = nh.subscribe("map", 1, &PolygonToGridConverter::mapCallback, this);
        polygon_sub = nh.subscribe("polygon_map", 1, &PolygonToGridConverter::polygonCallback, this);
        grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
        map_width = map_msg->info.width;
        map_height = map_msg->info.height;
        map_resolution = map_msg->info.resolution;

        // 在地图初始化后将多边形转换为网格
        if (!selected_polygon.points.empty())
        {
            convertPolygonToGrid(map_msg);
        }
    }

    void polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg)
    {
        selected_polygon = polygon_msg->polygon;
    }

    void convertPolygonToGrid(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
        nav_msgs::OccupancyGrid grid_map;
        grid_map.info = map_msg->info;
        grid_map.data.resize(map_width * map_height);

        for (int x = 0; x < map_width; ++x)
        {
            for (int y = 0; y < map_height; ++y)
            {
                geometry_msgs::Point point;
                point.x = x * map_resolution;
                point.y = y * map_resolution;

                if (isInsidePolygon(point, selected_polygon))
                {
                    // 将多边形内的网格单元标记为需要覆盖的区域
                    grid_map.data[y * map_width + x] = 100;  // 100 表示需要覆盖的区域
                }
                else
                {
                    grid_map.data[y * map_width + x] = -1;  // -1 表示不需要覆盖的区域
                }
            }
        }

        grid_pub.publish(grid_map);
    }

    bool isInsidePolygon(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon)
    {
        // 简单地检查点是否在多边形区域内（可根据需求调整）
        // 这里省略了实际的多边形内部判断算法
        // 可以使用射线法等方法判断点是否在多边形内部

        // 这里返回一个简单的示例结果
        return true;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Subscriber polygon_sub;
    ros::Publisher grid_pub;
    int map_width, map_height;
    double map_resolution;
    geometry_msgs::Polygon selected_polygon;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "polygon_grid_node");
    PolygonToGridConverter converter;

    ros::spin();

    return 0;
}
