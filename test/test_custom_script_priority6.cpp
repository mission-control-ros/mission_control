#include <iostream>
#include <stdio.h>
#include <string>
#include <mission_control/mission_control_utils.h>

int main (int argc, char* argv[])
{
  ros_init("test_custom_script_priority6", argc, argv);

  std::string name = "test_counter6";
  std::string def_val = "10";

  for(int i = 10; i >= 0; i--)
  {

    printf("In cpp file test_counter6 %d\n", i);
    set_var(name, std::to_string(i));
    sleep(1);

    std::string var = get_var(name, def_val);
    printf("In cpp got test_counter6 %s\n", var.c_str());
  }

  return 0;
}
