#include <iostream>
#include <stdio.h>
#include <string>
#include <mission_control/mission_control_utils.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

int main (int argc, char* argv[])
{
  ros_init("custom_script_priority6_set", argc, argv);

  ros::NodeHandle nh = ros::NodeHandle();

  ros::Publisher start_setting_pub = nh.advertise<std_msgs::Bool>("/mission_control/test/mission_control_utils/start_setting", QUEUE_SIZE);
  ros::Publisher answer_pub = nh.advertise<std_msgs::Int32>("/mission_control/test/mission_control_utils/answer_test_counter6", QUEUE_SIZE);
  ros::Publisher expired_answer_pub = nh.advertise<std_msgs::Int32>("/mission_control/test/mission_control_utils/expired_answer_test_counter6", QUEUE_SIZE);

  sleep(2);

  std_msgs::Bool msg;
  msg.data = true;
  start_setting_pub.publish(msg);

  ros::spinOnce();

  sleep(2);

  std::string answer = get_var("test_counter6", "-1");

  std_msgs::Int32 answer_msg;
  answer_msg.data = atoi(answer.c_str());
  answer_pub.publish(answer_msg);

  ros::spinOnce();

  sleep(10); //Expire var

  std::string expired_answer = get_var("test_counter6", "-1");

  std_msgs::Int32 expired_answer_msg;
  expired_answer_msg.data = atoi(expired_answer.c_str());
  expired_answer_pub.publish(expired_answer_msg);

  ros::spinOnce();

  sleep(2); //Wait for the message to reach the end

  return 0;
}
