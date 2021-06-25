# TurtleBot3(Note to self)

## Required Topics

| Topic Name |        Type         | Publisher | Subscriber |
| :--------: | :-----------------: | :-------: | :--------: |
|  /cmd_vel  | geometry_msgs/Twist |    Me     |   Turtle   |
|   /odom    |  nav_msgs/Odometry  |  Turtle   |     Me     |

## Types:

###  geometry_msgs/Twist

```json
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

### nav_msgs/Odometry

```json
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance

```



## Commands

**Run TurtleBot3:**  

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

**Compile Turtle Controller:**  

```bash
catkin_make
```

**Run Turtle Controller:**  

```bash
rosrun beginner_tutorials controller
```

If still it shows some error then run

```bash
source devel/setup.bash
```

