# mbari_wec_template_cpp
C++ template repo for external ROS2-enabled controller for MBARI's wave energy conversion buoy (sim or physical).

Please see [buoy_msgs/buoy_api_cpp](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_cpp)
for controller [examples](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_cpp/examples)

## Modify template for your package
Replace `mbari_wec_template_cpp` with your package name in

- package.xml
- CMakeLists.txt
- launch/controller.launch.py
- all `#include` paths for `controller.hpp` and `control_policy.hpp`

and rename the folder `include/mbari_wec_template_cpp` containing `controller.hpp` and `control_policy.hpp`

Modify `CMakeLists.txt` as desired and add any rosdep dependencies in `package.xml`.

## Implement Controller
Assuming you have followed the above and renamed everything from `mbari_wec_template_cpp` to your package name,
the following files are stubbed out to implement your custom external controller:
- `include/<your_package_name>/controller.hpp`
- `include/<your_package_name>/control_policy.hpp`
- `src/controller.cpp`

You may also use `config/controller.yaml` for any policy parameters.
If you modify the controller node name, i.e. `rclcpp::spin(std::make_shared<Controller>("controller"));`
and the `name` field of `Node` in the launch file,
please be sure to update the first line of the `config/controller.yaml` file and to use the same node name.

Also, if you choose a more specific name for your controller,
consider renaming:
- `launch/controller.launch.py`
- `config/controller.yaml`
- `include/<your_package_name>/controller.hpp`
- `include/<your_package_name>/control_policy.hpp`
- `src/controller.cpp`

and update:

- `CMakeLists.txt`
- `executable` field of `Node` in `launch/controller.launch.py`

accordingly.
