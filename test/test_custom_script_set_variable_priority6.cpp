#include <iostream>
#include <stdio.h>
#include <string>
#include <mission_control/mission_control_utils.h>
#include <std_msgs/Bool.h>

volatile bool stop_asking = false;

void stop_asking_callback(const std_msgs::Bool::ConstPtr& msg)
{
  stop_asking = true;
}

int main (int argc, char* argv[])
{
  ros_init("custom_script_priority6_set", argc, argv);

  ros::NodeHandle nh = ros::NodeHandle();

  ros::Publisher start_asking_pub = nh.advertise<std_msgs::Bool>("/mission_control/test/mission_control_utils/start_asking", QUEUE_SIZE);
  ros::Subscriber stop_asking_sub = nh.subscribe<std_msgs::Bool>("/mission_control/test/mission_control_utils/stop_asking", QUEUE_SIZE, stop_asking_callback);

  std::string test_counter6_1 = "15";

  sleep(5);

  std_msgs::Bool msg;
  msg.data = true;
  start_asking_pub.publish(msg);

  for(int i = 0; i <= 10; i++)
  {
    ros::spinOnce();
    set_var("test_counter6_1", test_counter6_1);
    ros::spinOnce();
    sleep(1);
  }

  return 0;
}
