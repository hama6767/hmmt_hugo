+++
date = '2025-02-10T10:50:46+02:00'
draft = false
title = 'Run a simulation'
math = true
weight = 3
+++


## How to Run the Simulation with a ROS 2 Launch File 🚀

A ROS 2 **Launch File** is a powerful tool for automating the startup and configuration of nodes. It allows you to run complex applications with a single, reproducible command.

This page breaks down a sample launch file to explain how it works and provides the steps to run your simulation.

-----

## Understanding the Launch File

Below is a typical launch file for starting the simulation. Let's break down what each part of the code does.

**`your_launch_file.launch.py`:**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    mujoco_pkg_share = get_package_share_directory('dr_mujoco')
    simulation_launch_path = os.path.join(
        mujoco_pkg_share, 'launch', 'default.launch.py'
    )

    package_path = get_package_share_directory('<your_package_name>')
    robot_model_path_arg = DeclareLaunchArgument(
        'robot_model',
        default_value=os.path.join(package_path, 'mujoco', '<your_robot_model>.xml')
    )

    simulation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={
            'robot': LaunchConfiguration('robot_model')
        }.items()
    )

    return LaunchDescription([
        robot_model_path_arg,
        simulation_include,
    ])
```

### 1\. Defining a Launch Argument (`DeclareLaunchArgument`)

This part is the key to the launch file's flexibility.

```python
    robot_model_path_arg = DeclareLaunchArgument(
        'robot_model',
        default_value=os.path.join(package_path, 'mujoco', '<your_robot_model>.xml')
    )
```

  - **`DeclareLaunchArgument`** defines an **argument** that can be passed from the command line when you run `ros2 launch`.
  - Here, we define an argument named `robot_model`.
  - The **`default_value`** is the value used automatically if no argument is provided. This lets you set a commonly used model as the default.
  - This feature allows you to change which model is loaded by simply adding `robot_model:=/path/to/another.xml` to your launch command.

### 2\. Including Another Launch File (`IncludeLaunchDescription`)

Launch files can call, or "include," other launch files to reuse functionality.

```python
    simulation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={
            'robot': LaunchConfiguration('robot_model')
        }.items()
    )
```

  - **`IncludeLaunchDescription`** is an action that starts another launch file (in this case, the main simulator, `default.launch.py`).
  - The most important part is **`launch_arguments`**. This specifies the arguments to pass to the included launch file.
  - The line `'robot': LaunchConfiguration('robot_model')` means: "Take the value of our local `robot_model` argument and pass it to the `robot` argument that `default.launch.py` expects."

This makes your launch file act as a "configuration bridge" between the main simulator and your specific robot model.

-----

## Steps to Run the Simulation

### Step 1: Prepare Your File and Folder Structure

To run correctly, your files must be placed in specific locations within your package.

```
colcon_ws/
└── src/
    └── <your_package_name>/
        ├── launch/
        │   └── <your_launch_file_name>.launch.py  <-- Save the code here
        ├── mujoco/
        │   └── <your_robot_model>.xml             <-- Your robot model
        └── ...
```

### Step 2: Build Your Workspace

After adding the new launch file, you need to build your workspace so ROS 2 can find it. Run this command from the root of your workspace (e.g., `colcon_ws/`).

```bash
colcon build --packages-select <your_package_name>
```

### Step 3: Source Your Environment

Once the build is complete, source the setup file to make your package available in the current terminal.

```bash
source install/setup.bash
```

### Step 4: Launch the Simulation

Now you are ready to go. Run the following command to start the simulation:

```bash
ros2 launch <your_package_name> <your_launch_file_name>.launch.py
```

If successful, the MuJoCo viewer window will open, displaying the robot defined in your `<your_robot_model>.xml` file.

-----

### (Advanced) Dynamically Specifying a Model from the Command Line

Thanks to `DeclareLaunchArgument`, you can switch models at runtime without editing any files.

```bash
ros2 launch <your_package_name> <your_launch_file_name>.launch.py \
  robot_model:='<path/to/your/other_robot.xml>'
```