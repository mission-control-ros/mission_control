#include <iostream>
#include <stdio.h>
#include <string>
#include <mission_control/mission_control_utils.h>

int main (int argc, char** argv)
{
  ros_init("mission_control_utils");

  std::string name = "counter6";
  std::string def_val = "10";

  for(int i = 0; i < 10; i++)
  {

    printf("In cpp file counter6 %d\n", i);
    set_var(name, std::to_string(i), 5);
    sleep(1);

    std::string var = get_var(name, def_val);
    printf("In cpp got counter6 %s\n", var.c_str());
  }

  for(int i = 0; i < 10; i++)
  {
    printf("In cpp file counter6 %d\n", i);
    sleep(1);
    std::string var = get_var(name, def_val);
    printf("In cpp got counter6 %s\n", var.c_str());
  }

  return 0;
}
