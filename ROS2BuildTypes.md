# Build types used in ROS2

to create a ROS2 package, we run `cd ros2_ws/src` then run the following command:

`
ros2 pkg create pkgname --build-type ament_python 
`

Notice how there is a `--build-type` flag here, we will look in to what does this flag expects as arguments.

## 🧩 1. ament_python
➡️ for pure python project (no c++ code).

__uses:__
- `setup.py` for installation
- `package.xml` for ROS metadata
- Dependencies go in both files

__Typical use case:__
- Nodes written in Python(`rclpy`)
- Utility or script-only packages
__Example Layout:__

```
my_pkg/
├── package.xml
├── setup.py
├── my_pkg/
│   └── my_node.py
└── resource/my_pkg
```

__Build Command:__

```
colcon build --packages-select my_png
```


## 🧩 2. ament_cmake
➡️ for c++ packages (or mixed c++ header libraries).

__uses:__
- `CMakeLists.txt` for build logic
- `package.xml` for for dependencies
- `ament_cmake` macros

__Typical use case:__
- Nodes written in C++(`rclcpp`)
- Packages that define libraries or executables
__Example Layout:__

```
my_cpp_pkg/
├── CMakeLists.txt
├── package.xml
└── src/
    └── my_node.cpp
```

## 🧩 3. ament_cmake_python
➡️ for hybrid packages (c++ and python).

__uses:__
- both `CMakeLists.txt` and `setup.py` 
- Links Python scripts and C++ extensions in one package

__Typical use case:__
- When you have a C++ backend library + Python frontend node e.g., C++ bindings exposed via Python.
__Example Layout:__

```
my_hybrid_pkg/
├── package.xml
├── setup.py
├── CMakeLists.txt
├── src/
│   └── cpp_part.cpp
└── my_hybrid_pkg/
    └── py_node.py
```


## 🧩 4. cmake
➡️ for Non-ROS CMake projects inside a ROS workspace.

__uses:__
- Standard `CMakeLists.txt` 
- Does _not_ use `ament` macros

__Typical use case:__
- Low-level libraries not depending on ROS 2
- Bringing external CMake projects into ROS workspace
__Example Layout:__

```
my_lib/
├── CMakeLists.txt
└── src/
    └── my_lib.cpp
```