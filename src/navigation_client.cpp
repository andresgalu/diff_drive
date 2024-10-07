#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <diff_drive/NavigateAction.h>

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "navigate_client_node");

  // Create action client
  actionlib::SimpleActionClient<diff_drive::NavigateAction> ac("Navigate", true);

  // wait for the action server to start
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  // Parse input arguments
  cout << "Argument count: " << argc << "\n";
  for (int i=0; i<argc; ++i)
    cout << "Argument[" << i << "]: " << argv[i] << "\n";
  cout << "\n";

  if (argc != 3)
  {
    cout << "Wrong number of arguments.\n";
    cout << "Please input goal X and Y coordinates of the robot.\n";
    return 0;
  }

  // send a goal to the action
  ROS_INFO("Action server started, sending goal.");
  diff_drive::NavigateGoal goal;
  goal.point.x = stod(argv[1]);
  goal.point.y = stod(argv[2]);
  goal.point.z = 0.;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

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