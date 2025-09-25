+++
date = '2025-02-10T10:50:46+02:00'
draft = false
title = 'Make your robot model'
math = true
weight = 3
+++


## How to Create MuJoCo XML Files

This is a step-by-step guide for beginners on how to create the MuJoCo XML files used in this simulator.

-----

## What is a MuJoCo XML File?

MuJoCo (**Mu**lti-**Jo**int dynamics with **Co**ntact) is an advanced physics simulation engine. The **XML file** is what defines the simulation environment, including the world, the robot's shape, its weight, and how its joints move.

For this simulator, you will primarily prepare two XML files:

  * `world.xml`: Defines the **environment**, such as the ground and lighting.
  * `robot.xml`: Defines the **robot model** that you will control in the simulation.

When the program starts, these two files are automatically merged to create a single simulation world.

-----

## Basic Structure of an XML File

An XML file is made up of elements enclosed in `<tag>` and `</tag>`. A MuJoCo XML file generally has the following structure:

```xml
<mujoco model="model_name">

  <asset>
    </asset>

  <worldbody>
    </worldbody>

  <actuator>
    </actuator>

</mujoco>
```

-----

## Step-by-Step Creation Guide

### Step 1: Create the World (`world.xml`)

First, let's build the environment where the robot will be placed. At a minimum, you need **ground** and a **light source**.

**Example `world.xml`:**

```xml
<mujoco model="my_world">
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>

    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.9 0.8 1"/>
  </worldbody>
</mujoco>
```

  * **`<light>`**: Illuminates the scene.
  * **`<geom>`**: Short for "geometry," this defines the shape of an object. Here, `type="plane"` creates the ground.
      * **`name`**: A unique name for the element.
      * **`type`**: The type of shape (`plane`, `box`, `sphere`, `cylinder`, etc.).
      * **`size`**: The dimensions of the shape.
      * **`rgba`**: The color and transparency (Red, Green, Blue, Alpha).

### Step 2: Create the Robot (`robot.xml`)

This is the main part\! We will assemble the robot piece by piece. A robot is built using a nested structure of `<body>` elements.

#### `<body>`: The Parts of the Robot

A `<body>` element represents a part of the robot, like the chassis, a wheel, or an arm. The `pos` attribute specifies its position relative to its parent body.

```xml
<body name="base_link" pos="0 0 0.05">
  <body name="left_wheel" pos="0 0.1 0">
    </body>

  <body name="right_wheel" pos="0 -0.1 0">
    </body>
</body>
```

#### `<joint>`: Connecting the Parts

Joints define how bodies are connected to each other (e.g., whether they rotate or slide).

```xml
<body name="left_wheel" pos="0 0.1 0">
  <joint name="base_to_left_wheel" type="hinge" axis="0 0 1"/>
  ...
</body>
```

  * **`name`**: The name of the joint. **This is crucial for linking it to a motor later.**
  * **`type`**: The type of joint.
      * `hinge`: A rotational joint (like a wheel or an elbow). `axis` defines the axis of rotation.
      * `slide`: A linear (sliding) joint.
      * `free`: Allows free movement and rotation in space. Often used for the root body of a robot.
  * **`axis`**: The axis of rotation or movement (X Y Z).

#### `<geom>`: Shape and Appearance

Add a `<geom>` to each body to define its physical shape and appearance.

```xml
<body name="left_wheel" pos="0 0.1 0">
  <joint ... />
  <geom name="left_wheel_geom" type="cylinder" size="0.03 0.01" rgba="0 0 0 1"/>
</body>
```

  * **`name`**: The name of the geometry. **The program uses this to get information.**
  * **`size`**: The size, which depends on the shape. For a cylinder, it's "radius height."
  * **`rgba`**: The color.

#### `<inertial>`: Mass and Weight

To make the physics simulation realistic, you must set the mass and inertia for each body.

```xml
<body name="base_link" pos="0 0 0.05">
  <joint type="free"/>
  <inertial pos="0 0 0" mass="0.1" diaginertia="0.01 0.01 0.01"/>
  ...
</body>
```


#### `<camera>`: The Visual Sensor 📷

You can attach a camera to your robot to get images from within the simulation. A camera is defined by adding a `<camera>` tag inside the `<body>` where you want to attach it.

**Example: Adding a camera to a "head" body**

```xml
<body name="head" pos="0.12 0 0.025">
  <geom name="head_geom" type="box" size="0.015 0.015 0.015"/>

  <camera name="front_camera" pos="0.015 0 0" euler="90 -90 0" fovy="60"/>
</body>
```

  * **`name`**: The camera's name. **This is mandatory for the program to retrieve the image stream. Set it to `front_camera`.**
  * **`pos`**: The relative position from the parent body (X Y Z).
  * **`euler`**: The camera's orientation in Euler angles (in degrees), which defines its rotation around different axes.
  * **`fovy`**: The vertical field of view. A larger value results in a wider-angle lens.

**Note:** Please be aware that you need to add an argument when launching the simulator to enable the camera stream.

#### `<actuator>`: The Driving Force

Actuators define the motors that power the joints. They are grouped together in the `<actuator>` section.

```xml
<actuator>
  <velocity name="left_wheel_vel" joint="base_to_left_wheel" kv="0.05"/>

  <velocity name="right_wheel_vel" joint="base_to_right_wheel" kv="0.05"/>
</actuator>
```

  * **`velocity`**: An actuator type that controls velocity.
  * **`name`**: The name of the actuator. **The program uses this for control.**
  * **`joint`**: **Specifies which joint to drive.** This must match the `name` of a `<joint>`.

-----

## Linking with the Program  **[MOST IMPORTANT]**

The simulator's Python code identifies and controls parts of the robot using the specific **names (`name` attribute)** you write in the XML file. Therefore, the `name` in your XML **must exactly match** the definitions in the code.

Here is the table showing the correspondence between the constants in the code and the `name` attributes in the XML.

| Python Constant Name       | Corresponding XML `name` Attribute | What it Refers To        |
|:---------------------------|:-----------------------------------|:-------------------------|
| `LEFT_WHEEL_GEOM`          | `left_wheel_geom`                  | Left wheel's `<geom>`    |
| `LEFT_WHEEL_BODY`          | `left_wheel`                       | Left wheel's `<body>`    |
| `RIGHT_WHEEL_BODY`         | `right_wheel`                      | Right wheel's `<body>`   |
| `FRONT_CAMERA`             | `front_camera`                     | The `<camera>`           |
| `LEFT_WHEEL_JOINT`         | `base_to_left_wheel`               | Left wheel's `<joint>`   |
| `RIGHT_WHEEL_JOINT`        | `base_to_right_wheel`              | Right wheel's `<joint>`  |
| `LEFT_WHEEL_ACTUATOR`      | `left_wheel_vel`                   | Left wheel's `<actuator>`|
| `RIGHT_WHEEL_ACTUATOR`     | `right_wheel_vel`                  | Right wheel's `<actuator>`|

**Example: Left Wheel Definition**

```xml
<body name="left_wheel" pos="0 0.175 0">

  <joint name="base_to_left_wheel" type="hinge" axis="0 0 1"/>

  <geom name="left_wheel_geom" type="cylinder" size="0.036 0.004"/>

</body>

...

<actuator>
  <velocity name="left_wheel_vel" joint="base_to_left_wheel" />
</actuator>
```

If this mapping is incorrect, the simulator will fail to start with an error like "`Could not find model component`."

-----

## Troubleshooting

  * **Error: `Could not find model component: ...`**

      * **Cause:** The Python code cannot find a `name` that it's looking for in your XML file.
      * **Solution:** Check the table above and make sure all the `name` attributes in your XML file are correct. Check for typos.

  * **The robot doesn't move.**

      * **Cause:** The `<actuator>` configuration might be wrong.
      * **Solution:** Ensure the `joint` attribute in your `<actuator>` tag exactly matches the `name` of the `<joint>` you want to move.

  * **The robot sinks into the ground or flies away.**

      * **Cause:** The physics properties (like mass in `<inertial>` or contact parameters in `<geom>`) might be inappropriate.
      * **Solution:** Try adjusting the mass and other values, using the provided sample XML as a reference.

