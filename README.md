# mbari_wec_template_cpp
C++ template repo for external ROS2-enabled controller for MBARI's wave energy conversion buoy (sim or physical).

Please see [mbari_wec](https://github.com/osrf/mbari_wec) and associated
[documentation](https://osrf.github.io/mbari_wec). There is also a
[Python3](https://github.com/mbari-org/mbari_wec_template_py) version of the template repository.

Please also see the
[linear_damper_example](https://github.com/mbari-org/mbari_wec_template_cpp/tree/linear_damper_example)
branch of this template repository or
[mbari_wec_utils/buoy_api_cpp](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp)
for controller [examples](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp/examples)

## Tutorial
There is a [tutorial](https://osrf.github.io/mbari_wec/Tutorials/ROS2/CppTemplate/) for a quick start
using this template. You may also refer to GitHub's
[template documentation](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)
for general help with template repositories.

## Modify template for your package
1. Click the green
button with the text `Use this template` and select `Create a new repository`

2. Next, set up the repository like you would any new GitHub repository choosing the owner,
repository name, public/private, etc.

3. Now that your new repository is set up, clone it to your ROS 2 workspace on your local machine,
make a branch, etc.  
   `$ git clone https://github.com/<owner>/<repo_name>.git`

You should now have a C++ ROS 2 package with the following structure in your workspace:

```
<repo_name>
    ├── CMakeLists.txt
    ├── config
    │   └── controller.yaml
    ├── include
    │   └── mbari_wec_template_cpp
    │       ├── controller.hpp
    │       └── control_policy.hpp
    ├── launch
    │   └── controller.launch.py
    ├── LICENSE
    ├── package.xml
    ├── README.md
    └── src
        └── controller.cpp
```

Replace `mbari_wec_template_cpp` with your package name and other fields as necessary in:

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

## Example

An example using this interface can be found in the tutorial:
[Linear Damper Example (C++)](https://osrf.github.io/mbari_wec/Tutorials/ROS2/CppLinearDamperExample.md)
