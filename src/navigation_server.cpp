#include <math.h>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <diff_drive/NavigateAction.h>

class NavigateAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<diff_drive::NavigateAction> as_; 
  std::string action_name_;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  geometry_msgs::TransformStamped robot_transform_msg;
  ros::Publisher command_pub;

  // Output messages
  diff_drive::NavigateFeedback feedback_;
  diff_drive::NavigateResult result_;
  geometry_msgs::Twist command;

  // Parameters
  double angle_tolerance = 0.008726646;     // Max angle considered correct: 0.5deg
  double distance_tolerance = 0.1;          // Max distance considered correct: 10cm
  double max_linear_speed = 0.4;            // Max linear speed: 40cm/s
  double max_angular_speed = 0.3490658;     // Max angular speed: 20º/s
  double linear_near_threshold = 1.2;       // Distances below are considered near
  double angular_near_threshold = 0.1745329;// Angles below are considered near

public:

  NavigateAction(std::string name) :
    as_(nh_, name, boost::bind(&NavigateAction::executeCB, this, _1), false),
    action_name_(name),
    tf_listener(tf_buffer)
  {
    as_.start();
    command_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Initialize command to 0
    setCommandToZero();
  }

  ~NavigateAction(void)
  {
  }

  void executeCB(const diff_drive::NavigateGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(10);
    bool success = false;

    // Start measuring time
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Check for the existence of frames
    for (int i=0; i<5; ++i)
    {
      // Wait for a while if they cant be found
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (tf_buffer._frameExists("demo") && tf_buffer._frameExists("diff_drive"))
        break;
      else
        std::cout << "tf2 could not find the required frames.\n";
    }

    // Go over all waypoints in order
    int n_wps = goal->waypoints.size();
    std::cout << "\nReceived request to follow " << n_wps << " waypoints\n";
    for (int i=0; i<n_wps; ++i)
    {
      success = false;
      // start executing the action
      while(success == false)
      {
        std::cout << "\nThe goal is: ";
        std::cout << goal->waypoints[i].x << ", ";
        std::cout << goal->waypoints[i].y << "\n";

        // Check current position of robot
        robot_transform_msg = tf_buffer.lookupTransform("demo", "diff_drive", ros::Time(0));
        tf2::Transform robot_transform;
        tf2::fromMsg(robot_transform_msg.transform, robot_transform);
        std::cout << "The robot position is: ";
        std::cout << robot_transform_msg.transform.translation.x << ", ";
        std::cout << robot_transform_msg.transform.translation.y << "\n";

        // Work in local coordinates
        tf2::Vector3 point_global(goal->waypoints[i].x, goal->waypoints[i].y, goal->waypoints[i].z);
        tf2::Vector3 point_local = robot_transform.inverse()(point_global);
          // Calculate distance to goal
        double distance = sqrt(pow(point_local.getX(), 2) + pow(point_local.getY(), 2));
          // Calculate angle to goal
        double rotation_angle = atan2(point_local.getY(), point_local.getX());
        rotation_angle = remainder(rotation_angle, 2*M_PI);
        std::cout << "Robot needs to rotate " << rotation_angle*180./M_PI << "º\n";
          // Calculate the distance along X (local)
        double x_distance = point_local.getX();
        std::cout << "Robot needs to continue " << x_distance << "m\n";

        // Calculate velocity command
          // Linear velocity
        bool linear_success = false;
        if (abs(x_distance) > distance_tolerance)
        {
          if (abs(x_distance) > linear_near_threshold)
            command.linear.x = copysign(max_linear_speed, x_distance);
          else
            command.linear.x = copysign(max_linear_speed * abs(x_distance)/linear_near_threshold, x_distance);        // Linear
            //command.linear.x = copysign(max_linear_speed * sqrt(abs(x_distance)/linear_near_threshold), x_distance);  // Sqrt
        }
        else
        {
          linear_success = true;
          command.linear.x = 0.;
        }

          // Rotation
        bool rotation_success = false;
        if (abs(rotation_angle) > angle_tolerance)
        {
          if (abs(rotation_angle) > angular_near_threshold)
            command.angular.z = copysign(max_angular_speed, rotation_angle);
          else
            command.angular.z = copysign(max_angular_speed * abs(rotation_angle)/angular_near_threshold, rotation_angle);
        }
        else
        {
          rotation_success = true;
          command.angular.z = 0.;
        }

        // Check if success
        //success = linear_success && rotation_success;   // rotation_success doesnt make much sense when close to target
        success = distance < distance_tolerance;

        // Publish velocity command
        command_pub.publish(command);
        std::cout << "Velocity command:\n";
        std::cout << "vx: " << command.linear.x << ", wz: " << command.angular.z << "\n";

        // Fill feedback
        feedback_.distance_remaining = distance;
        feedback_.angle_remaining = rotation_angle;
        feedback_.waypoints_remaining = n_wps - i;


        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
          break;
        }
        // publish the feedback
        as_.publishFeedback(feedback_);
        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep();
      }
    }

    // Clear the velocity command
    setCommandToZero();
    command_pub.publish(command);

    // Stop measuring time
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double time_elapsed = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

    // Send result message
    if(success)
    {
      result_.message = "Success";
      result_.time_elapsed = time_elapsed;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

  void setCommandToZero()
  {
    command.linear.x = 0.;
    command.linear.y = 0.;
    command.linear.z = 0.;
    command.angular.x = 0.;
    command.angular.y = 0.;
    command.angular.z = 0.;
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Navigate");

  NavigateAction Navigate("Navigate");
  ros::spin();

  return 0;
}