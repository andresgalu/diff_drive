#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// Define a class to handle the LiDAR subscription and callback
class ObstacleAvoider
{
public:
    // Constructor
    ObstacleAvoider(ros::NodeHandle& nh)
    {
        // Subscribe to the LiDAR scan topic
        front_lidar_sub_ = nh.subscribe("/front_lidar", 10, &ObstacleAvoider::frontLidarCallback, this);
        rear_lidar_sub_ = nh.subscribe("/rear_lidar", 10, &ObstacleAvoider::rearLidarCallback, this);

        // Subscribe to the velocity command
        cmd_vel_sub_ = nh.subscribe("/cmd_vel_orig", 10, &ObstacleAvoider::commandCallback, this);

        // Advertise publisher
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    }

private:
    // Parameters
    float range_threshold = 1.0;
    bool can_move_forward = false;
    bool can_move_backward = false;

    // Subscribers
    ros::Subscriber front_lidar_sub_;
    ros::Subscriber rear_lidar_sub_;
    ros::Subscriber cmd_vel_sub_;

    // Publisher
    ros::Publisher cmd_vel_pub_;

    // Find min value in scan
    float findMinRange(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        std::vector<float> ranges = scan->ranges;
        std::vector<float>::iterator min_elem = std::min_element(ranges.begin(), ranges.end());
        return *min_elem;
    }

    // Callback function to process the front lidar data
    void frontLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        // Find minimum value
        float min_range = findMinRange(scan);
        
        // Check if inside danger zone
        can_move_forward = min_range > range_threshold;
    }

    // Callback function to process the rear lidar data
    void rearLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        // Find minimum value
        float min_range = findMinRange(scan);
        
        // Check if inside danger zone
        can_move_backward = min_range > range_threshold;
    }

    // Callback function to process the cmd_vel
    void commandCallback(const geometry_msgs::Twist::ConstPtr& cmd_in)
    {
        // Create new command
        geometry_msgs::Twist cmd_out = *cmd_in;

        // Check if movement is possible
        if (cmd_out.linear.x > 0 && !can_move_forward)
            cmd_out.linear.x = 0.;
        if (cmd_out.linear.x < 0 && !can_move_backward)
            cmd_out.linear.x = 0.;

        // Send command
        cmd_vel_pub_.publish(cmd_out);
    }
};

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "obstacle_avoidance");

    // Create a node handle
    ros::NodeHandle nh;

    // Instantiate the LidarSubscriber class
    ObstacleAvoider avoider(nh);

    // Spin to keep the node alive and listening for messages
    ros::spin();

    return 0;
}