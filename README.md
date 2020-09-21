# ROS Dynamic Reconfigure

This repository demonstrates the usage of dynamic reconfigure your ROS package.  

## cfg File

**Step 1:**  
Create a cfg direcotry inside your package and make a cfg file.  
In order to make this cfg file usable it must be executable.  
```bash
mkdir cfg
touch cfg/RosDynamicReconfigure.cfg
chmod a+x cfg/RosDynamicReconfigure.cfg
```

Define your dynamic reconfiguration parameters in the cfg file.  
Remember your file name should not consist of hyphen or dashes, as they are not supported by c++.  
```python
#!/usr/bin/env python

# Define your package name here, due to the fact that hyphen and dashes is not used for variable name, use underscore instead.  
PACKAGE="ros_dynamic_reconfigure"

from dynamic_reconfigure.parameter_generator_catkin import *


# Parameter generator handle
gen = ParameterGenerator()

# Populate drop-down menu content
# Name | Type | Value | Description
drop_down_menu_1 = gen.enum([
    gen.const("default_value", int_t, -1, "Not used"),
    gen.const("value_zero", int_t, 0, "This is value zero"),
    gen.const("value_one", int_t, 1, "This is value one")
])

drop_down_menu_2 = gen.enum([
    gen.const("defaul_value", str_t, "default value", "default value"),
    gen.const("hello_world", str_t, "hello world!", "hello"),
    gen.const("goodbye_world", str_t, "goodbye world!", "goodbye")
])

# Add drop-down menu to window
# Name | Type | Level | Description | Default | Min | Max | Values
gen.add("drop_down_menu_1", int_t, 0, "Drop_down Menu 1", -1, -1, 1, edit_method=drop_down_menu_1)
gen.add("drop_down_menu_2", str_t, 0, "Drop_down Menu 2", edit_method=drop_down_menu_2)

# Add boolean option to window
# Name | Type | Level | Description | Default | Min | Max | Values
gen.add("true_or_false", bool_t, 0, "True or False", True)

# Add double option to window
# Name | Type | Level | Description | Default | Min | Max | Values
gen.add("slider_points", double_t, 0, "Slider", 0.7, 0, 1)

# Exit parameter generator
# Package Name | Node Name | cfg File Name
exit(gen.generate(PACKAGE, "RosDynamicReconfigure", "RosDynamicReconfigure"))
```