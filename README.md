# ROS Dynamic Reconfigure

This repository demonstrates the usage of dynamic reconfigure your ROS package.  
Something to take note is that my package name has hyphen and dashes but when writing a cpp code you will not be allowed to use them. Therefore, follow the instructions carefully to change the hyphen and dashes to underscore.  

## Usage

Source your workspace and select your dynamic_reconfigure tab.  
```bash
source ~/my_ws/devel/setup.bash
rosrun rqt_reconfigure rqt_reconfigure
```

## cfg File

**Step 1:**  
Create a cfg direcotry inside your package and make a cfg file.  
In order to make this cfg file usable it must be executable.  
```bash
mkdir cfg
touch cfg/RosDynamicReconfigure.cfg
chmod a+x cfg/RosDynamicReconfigure.cfg
```

**Step 2:**  
Define your dynamic reconfiguration parameters in the cfg file.  
Remember your file name should not consist of hyphen or dashes, as they are not supported by c++.  
```python
#!/usr/bin/env python

# Define your cpp namespace here, due to the fact that hyphen and dashes is not used for variable name, use underscore instead.  
CPPNAMESPACE="ros_dynamic_reconfigure"

from dynamic_reconfigure.parameter_generator_catkin import *


# Parameter generator handle
gen = ParameterGenerator()

# Populate drop-down menu content
# Name | Type | Value | Description
drop_down_menu = gen.enum([
    gen.const("default_value", int_t, -1, "Not used"),
    gen.const("value_zero", int_t, 0, "This is value zero"),
    gen.const("value_one", int_t, 1, "This is value one")],
    "Drop_down Menu"
)

# Add drop-down menu to window
# Name | Type | Level | Description | Default | Min | Max | Values
gen.add("drop_down_menu_1", int_t, 0, "Drop_down Menu 1", -1, -1, 1, edit_method=drop_down_menu)

# Add boolean option to window
# Name | Type | Level | Description | Default | Min | Max | Values
gen.add("true_or_false", bool_t, 0, "True or False", True)

# Add double option to window
# Name | Type | Level | Description | Default | Min | Max | Values
gen.add("slider_points", double_t, 0, "Slider", 0.7, 0, 1)

# Exit parameter generator
# Package Name | Node Name | cfg File Name
exit(gen.generate(CPPNAMESPACE, "RosDynamicReconfigure", "RosDynamicReconfigure"))
```

**Step 3:**  
Allowing intellisense to work for ease of programming.  
Edit CMakeLists.txt and catkin_make.  
```cmake
# Generate dynamic reconfigure parameters in the 'cfg' folder
# Must be called before catkin_package()
generate_dynamic_reconfigure_options(
  cfg/RosDynamicReconfigure.cfg
)
```

**Step 4:**  
Include the generated config file in your node.  
```cpp
#include <ros-dynamic-reconfigure/RosDynamicReconfigureConfig.h>
```

**Step 5:**
Create a server handle and callback function.  
```cpp
    // Dynamic reconfigure handle
    // For server that is using nodehandler, please make sure that the nodehandler is a private one
    // ros::NodeHandle private_nh_("~");
    // dynamic_reconfigure::Server<ros_dynamic_reconfigure::RosDynamicReconfigureConfig> server(private_nh_);
    // For more detail please see the cpp code
    dynamic_reconfigure::Server<ros_dynamic_reconfigure::RosDynamicReconfigureConfig> server;

    // Set callback function for dynamic reconfigure (using lambda)
    server.setCallback(
        [this](ros_dynamic_reconfigure::RosDynamicReconfigureConfig & config, uint32_t level)
        {
            ROS_INFO_STREAM(
                "\nThe first drop-down: " << config.drop_down_menu <<
                "\nTrue of false: " << ((config.true_or_false) ? "true" : "false") <<
                "\nSlider value: " << config.slider_points << std::endl
            );
        });

    // ROS spin (wait for server to enter callback)
    ros::spin();

```

**Step 6:**  
Add dependecies to `library` or `executable` in CMakeLists.txt
```cmake
add_dependencies(dynamic_node ${PROJECT_NAME}_gencfg)
```
Another method is:
```cmake
add_dependencies(dynamic_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

## Error

If you encounter error message such as this:
```bash
PluginManager._load_plugin() could not load plugin "rqt_reconfigure/Param":
Traceback (most recent call last):
  File "/opt/ros/melodic/lib/python2.7/dist-packages/qt_gui/plugin_handler.py", line 102, in load
    self._load()
  File "/opt/ros/melodic/lib/python2.7/dist-packages/qt_gui/plugin_handler_direct.py", line 55, in _load
    self._plugin = self._plugin_provider.load(self._instance_id.plugin_id, self._context)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/qt_gui/composite_plugin_provider.py", line 72, in load
    instance = plugin_provider.load(plugin_id, plugin_context)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/qt_gui/composite_plugin_provider.py", line 72, in load
    instance = plugin_provider.load(plugin_id, plugin_context)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rqt_gui_py/ros_py_plugin_provider.py", line 61, in load
    return super(RosPyPluginProvider, self).load(plugin_id, plugin_context)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/qt_gui/composite_plugin_provider.py", line 72, in load
    instance = plugin_provider.load(plugin_id, plugin_context)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rqt_gui/ros_plugin_provider.py", line 106, in load
    return class_ref(plugin_context)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rqt_reconfigure/param_plugin.py", line 51, in __init__
    self._plugin_widget = ParamWidget(context)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rqt_reconfigure/param_widget.py", line 105, in __init__
    self, rp, self.sig_sysmsg
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rqt_reconfigure/node_selector_widget.py", line 102, in __init__
    self._update_nodetree_pernode()
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rqt_reconfigure/node_selector_widget.py", line 341, in _update_nodetree_pernode
    TreenodeQstdItem.NODE_FULLPATH
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rqt_reconfigure/treenode_qstditem.py", line 94, in __init__
    self._set_param_name(grn_current_treenode)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rqt_reconfigure/treenode_qstditem.py", line 246, in _set_param_name
    self._toplevel_treenode_name = self._list_treenode_names[0]
IndexError: list index out of range
```

Note that most probably your nodehandler namespace have some error. Please check properly how your initiate your node handle and did you pass the correct node handle to the dynamic server.

## Important Note

Note that when the code run to the set callback function for dynamic server, it will run once the callback function. If you read your params before your setting your callback function it will overwrite it.

Recommendation: Run `private_nh_.param()` after `dyn_srv_->setCallback()`.

This is to aviod overwritting the params that you have loaded. If you wish to see the effect you can checkout to the `rosparam_version` branch.

## Conclusion

Here this is just a simple demostration of using the dynamic_reconfigure server, you can update your parameters here inside the lambda function. The lambda function is perferred as the boost::bind is considered to be slower. For more information please watch this [video](https://www.youtube.com/watch?v=ZlHi8txU4aQ).  
