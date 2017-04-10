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

    write_debug("Setting variable named " + name + " with value " + msg->value, 2);

    last_upt[name] = ros::Time::now();

    if(msg->ttl > 0)
    {
      ttl[name] = ros::Duration(msg->ttl);
      write_debug("Setting variable named " + name + " with time to live " + std::to_string(msg->ttl), 3);
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

  write_debug("Publishing variable named " + name + " with value " + value + " with time to live " + std::to_string(ttl), 3);

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
    write_debug("Found variable named " + name + " in cache", 2);

    std::map<std::string, CallbackData>::iterator it = cch.find(name);

    struct CallbackData data = it->second;
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
    write_debug("Maximum callbacks for get_var function reached, setting variable named " + name + " with default value " + def_val, 2);

    struct CallbackData data;
    data.str_data = def_val;
    data.str_data_def = true;
    cch[name] = data;

    last_upt[name] = ros::Time::now();

    return def_val;
  }

  ros::spinOnce();

  write_debug("Asking for variable named " + name + " (" + std::to_string(counter) + "/" + std::to_string(MAX_CBS) + ")", 3);

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
    write_debug("Deleting variable named " + name + " from cache", 1);

    if(cch.find(name) != cch.end())
    {
      cch.erase(name);
      write_debug("Deleting variable named " + name + " from cch", 3);
    }

    if(_cch.find(under_name) != _cch.end())
    {
      cch.erase(under_name);
      write_debug("Deleting variable named _" + name + " from _cch", 3);
    }

    if(ttl.find(name) != ttl.end())
    {
      ttl.erase(name);
      write_debug("Deleting time to live for variable named " + name + " from ttl", 3);
    }

    if(last_upt.find(name) != last_upt.end())
    {
      last_upt.erase(name);
      write_debug("Deleting last update time for variable named " + name + " from last_upt", 3);
    }
  }
}

// Sends out message to all listening nodes about variable request.
void publish_get_var(std::string name)
{
  std_msgs::String msg;

  msg.data = name;

  get_pub.publish(msg);

  write_debug("Publishing getting message for variable named " + name, 3);
}

//Writes node's script/object debug message
void write_debug(std::string msg, int level)
{
  if(debug_level >= level)
    ROS_INFO_STREAM(node_parent_name << " script/object - " << msg);
}

// Initializes ROS node and necessary publishers/subscribers for scripts
void ros_init(std::string name, int argc, char* argv[])
{
  ros::init(argc, argv, name, ros::init_options::AnonymousName);

  ros::NodeHandle nh = ros::NodeHandle();

  set_pub = nh.advertise<mission_control::Variable>(VAR_SET_TOPIC, QUEUE_SIZE);
  get_pub = nh.advertise<std_msgs::String>(VAR_GET_TOPIC, QUEUE_SIZE);

  set_sub = nh.subscribe<mission_control::Variable>(VAR_SET_TOPIC, QUEUE_SIZE, set_var_cb);

  if(argc == 3)
  {
    debug_level = atoi(argv[1]);
    node_parent_name = argv[2];
  }

  write_debug("ROS init", 1);
}
