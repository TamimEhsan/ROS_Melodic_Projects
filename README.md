# ROS_Melodic_Tutorials

This is my self note for learning ROS Melodic

## Primary

ROS Melodic Morenia is primarily targeted at the Ubuntu 18.04 (Bionic) release, though other Linux systems as well as Mac OS X, Android, and Windows are supported to varying degrees. I am using Ubuntu 18.04 (Bionic) release in Virtual Machine.

You can find the installation procedure here: [Installation Guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Creating a Catkin Package

Create a package with 

```bash
catkin_create_pkg <package_name> <dependency1> <dependency2>
```

eg.

```bash
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

compileproject

```bash
catkin_make	
```

## Nodes

A node really isn't much more than an executable file within a ROS  package.  ROS  nodes use a ROS client library to communicate with other  nodes.  Nodes can publish or subscribe to a Topic.  Nodes can also  provide or use a Service.   

## Topics

![](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key.png)

Topics are kind of like a bridge between a publishing node and  a subscribing node. A publisher and  a subscriber has to communicate through a topic.

## Commands

All commands are to be executed in catkin workspace

**Start ROS server**

```bash
roscore
```

**Find all active topics**

```bash
rostopic list
```

**See details of a topic**

```bash
rostopic info <topic_name>
```

eg

```bash
rostopic info /cmd_vel
```

**See type used in a topic**

```bash
rosmsg show <topic_name>
```

eg

```bash
rosmsg show /cmd_vel
```

