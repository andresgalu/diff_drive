#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <diff_drive/NavigateAction.h>
#include <geometry_msgs/Point.h>

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "navigate_client_node");
  

  // Prepare waypoints structure
  vector<geometry_msgs::Point> waypoints;

  // Parse input arguments
  if (argc == 2)
  {
    // Assuming input is a file (string)
    string filename = argv[1];
      // Check if filename is valid
    if (filename.empty())
    {
      cout << "Input is an empty string. Please introduce file with waypoints.\n";
      return 0;
    }
    ifstream file(filename);

    // Read line by line
    string line;
    if (file.is_open())
    cout << "Reading file: " << filename << "\n"; 
    {
      while (getline(file, line))
      {
        // Parse line
        if (line.empty())
          break;
        else
        {
          // Split string
          string xstr = line.substr(0, line.find(","));
          string ystr = line.substr(line.find(",")+1, line.size());
          // To numbers
          geometry_msgs::Point point;
          point.x = stod(xstr);
          point.y = stod(ystr);
          waypoints.push_back(point);
        }
      }
    }
  }
  else if (argc == 3)
  {
    // Assuming input is a pair of points
    geometry_msgs::Point point;
    point.x = stod(argv[1]);
    point.y = stod(argv[2]);
    waypoints.push_back(point);
  }
  else
  {
    // Error in the number of arguments
    cout << "Wrong number of arguments. Either input the name of a file with waypoints, or a XY point.\n";
    return 0;
  }
  

  // Create action client
  actionlib::SimpleActionClient<diff_drive::NavigateAction> ac("Navigate", true);

  // wait for the action server to start
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();


  // send a goal to the action
  ROS_INFO("Action server started, sending goal.");
  diff_drive::NavigateGoal goal;
  goal.waypoints = waypoints;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
    ac.cancelGoal();
  }

  //exit
  return 0;
}