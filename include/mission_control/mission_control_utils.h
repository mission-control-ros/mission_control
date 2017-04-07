#ifndef CACHE_H_
#define CACHE_H_

#include <iostream>
#include <unistd.h>
#include <string>
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mission_control/Variable.h>

#define MAX_CBS 4

#define VAR_RECHECK_DELAY 0.5

#define VAR_GET_TOPIC "/mission_control/variable/get"

#define VAR_SET_TOPIC "/mission_control/variable/set"

#define QUEUE_SIZE 10

/**
  Holds different types of variable data to be stored into cache
*/
struct CallbackData
{
  CallbackData() : str_data_def(false), int_data_def(false), float_data_def(false), bool_data_def(false) {}
  std::string str_data;
  bool str_data_def;
  int int_data;
  bool int_data_def;
  double float_data;
  bool float_data_def;
  bool bool_data;
  bool bool_data_def;
};

/**
  Publisher for setting variables
*/
static ros::Publisher set_pub;
/**
  Publisher for getting variables
*/
static ros::Publisher get_pub;

/**
  Subscriber for receiving set variables
*/
static ros::Subscriber set_sub;

/**
  Holds variables time to live in seconds. Key - variable's name, value - time to live in  seconds
*/
static std::map<std::string, ros::Duration> ttl;

/**
  Holds time when variable was last updated. Key - variable's name, value - time last updated
*/
static std::map<std::string, ros::Time> last_upt;

/**
  Holds variables data. Key - variable's name, value - variable's value
*/
static std::map<std::string, CallbackData> cch;

/**
  Holds information which variables to store in cache. Key - variable's name with underscore prefix, value - boolean true 
*/
static std::map<std::string, bool> _cch;

/**
  Initializes ROS node and necessary publishers/subscribers for scripts 
    
  @param name Name for the created node
*/
void ros_init(std::string name);

/**
  Sends out message to all listening nodes about new variable value.

  @param name Variable's name
  @param value Variable's value
*/
void set_var(std::string name, std::string value);

/**
  Sends out message to all listening nodes about new variable value.

  @param name Variable's name
  @param value Variable's value
  @param ttl Variable's validity in seconds
*/
void set_var(std::string name, std::string value, int ttl);

/**
  Deals with incoming set variable msg
    
  If variable exists in cache then the value is updated otherwise it's ignored
  This callback is used only if script calls out ros_init 
  to make it into separate ROS node
    
  @param msg  msg.name is the variable's name, msg.value is the variable's value, msg.ttl is variable's validity
*/
void set_var_cb(const mission_control::Variable::ConstPtr& msg);

/**
  Sends out message to all listening nodes about variable request.

  @param name Variable's name
*/
void publish_get_var(std::string name);

/**
  Checks if the variable in cache is new enough or needs to be deleted

  @param name Variable's name
*/
void check_var(std::string name);

/**
  Request given variable's value

  Firstly checks if variable value is new enough
  Secondly checks cache, if variable exists there.
  Lastly, if variable is not in cache, requests variable from all initialized nodes

  @param name Requested variable's name
  @param def_val Default value when requested variable is not found

  @return requested variable's value
*/
std::string get_var(std::string name, std::string def_val);

/**
  Request given variable's value

  Firstly checks if variable value is new enough
  Secondly checks cache, if variable exists there.
  Lastly, if variable is not in cache, requests variable from all initialized nodes

  @param name Requested variable's name
  @param def_val Default value when requested variable is not found
  @param counter Number of times function is in recursion

  @return requested variable's value
*/
std::string get_var(std::string name, std::string def_val, int counter);

#endif /* CACHE_H_ */
