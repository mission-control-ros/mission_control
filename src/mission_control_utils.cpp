#include <mission_control/mission_control_utils.h>

// Deals with incoming set variable msg. If variable exists in cache then the value is updated otherwise it's ignored.
// This callback is used only if script calls out ros_init to make it into separate ROS node
void set_var_cb(const mission_control::Variable::ConstPtr& msg)
{
  std::string name = msg->name;
  std::string under_name = "_" + name;

  if(cch.find(name) != cch.end() || _cch.find(under_name) != _cch.end())
  {
    struct CallbackData data;
    data.str_data = msg->value;
    data.str_data_def = true;
    cch[name] = data;

    last_upt[name] = ros::Time::now();

    if(msg->ttl > 0)
    {
      ttl[name] = ros::Duration(msg->ttl);
    }
  }

  ros::spinOnce();
}

// Sends out message to all listening nodes about new variable value.
void set_var(std::string name, std::string value)
{
  set_var(name, value, 0);
}

// Sends out message to all listening nodes about new variable value.
void set_var(std::string name, std::string value, int ttl)
{
  mission_control::Variable msg;
  msg.name = name;
  msg.value = value;
  msg.ttl = ttl;
  set_pub.publish(msg);

  ros::spinOnce();
}

// Request given variable's value. Firstly checks if variable value is new enough.
// Secondly checks cache, if variable exists there. Lastly, if variable is not in cache, requests variable from all initialized nodes.
std::string get_var(std::string name, std::string def_val)
{
  return get_var(name, def_val, 0);
}

// Request given variable's value. Firstly checks if variable value is new enough.
// Secondly checks cache, if variable exists there. Lastly, if variable is not in cache, requests variable from all initialized nodes.
std::string get_var(std::string name, std::string def_val, int counter)
{
  ros::spinOnce();

  check_var(name);

  if(cch.find(name) != cch.end())
  {
    std::map<std::string, CallbackData>::iterator it = cch.find(name);

    struct CallbackData data = it->second;
    ROS_INFO_STREAM("Oli siin " << data.str_data);
    if(data.str_data_def)
    {
      return data.str_data;
    }
  }

  if(counter == 0)
  {
    _cch["_"+name] = true;
  }

  publish_get_var(name);

  if(counter > MAX_CBS)
  {
    struct CallbackData data;
    data.str_data = def_val;
    data.str_data_def = true;
    cch[name] = data;

    last_upt[name] = ros::Time::now();

    return def_val;
  }

  ros::spinOnce();

  sleep(VAR_RECHECK_DELAY);

  ros::spinOnce();

  return get_var(name, def_val, ++counter);
}

// Checks if the variable in cache is new enough or needs to be deleted
void check_var(std::string name)
{
  if(last_upt.find(name) == last_upt.end() || ttl.find(name) == ttl.end())
    return;

  ros::Duration var_ttl = ttl[name];
  ros::Time var_last_up = last_upt[name];
  std::string under_name = "_" + name;

  if(ros::Time::now() > (var_last_up + var_ttl))
  {
    if(cch.find(name) != cch.end())
        cch.erase(name);

    if(_cch.find(under_name) != _cch.end())
        cch.erase(under_name);

    if(ttl.find(name) != ttl.end())
        ttl.erase(name);

    if(last_upt.find(name) != last_upt.end())
        last_upt.erase(name);
  }
}

// Sends out message to all listening nodes about variable request.
void publish_get_var(std::string name)
{
  std_msgs::String msg;

  msg.data = name;

  get_pub.publish(msg);
}

// Initializes ROS node and necessary publishers/subscribers for scripts
void ros_init(std::string name)
{
  int argc; 
  char** argv;

  ros::init(argc, argv, name, ros::init_options::AnonymousName);

  ros::NodeHandle nh = ros::NodeHandle();

  set_pub = nh.advertise<mission_control::Variable>(VAR_SET_TOPIC, QUEUE_SIZE);
  get_pub = nh.advertise<std_msgs::String>(VAR_GET_TOPIC, QUEUE_SIZE);

  set_sub = nh.subscribe<mission_control::Variable>(VAR_SET_TOPIC, QUEUE_SIZE, set_var_cb);
}
