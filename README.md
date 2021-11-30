# Episode 0 - A Study in ROSe

## Introduction

**ROS**, which means the Robot Operating System, is a set of software libraries and tools to help you build robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. The point of ROS is to create a **robotics standard**, so you don't need to reinvent the wheel anymore when building new robotic software.

<img src="W0_Images/ROS_logo.png ">

### Main Objectives of this Workshop:
1. The objective of this course is to give you the basic tools and knowledge to be able to understand and create any basic ROS related project. You will be able to move robots, read their sensor data, make the robots perform intelligent tasks, see visual representations of complex data such as laser scans and debug errors in the programs.
1. This will allow you to understand the packages that others have done. So you can take ROS code made by others and understand what is happening and how to modify it for your own purposes
1. This can serve as an introduction to be able to understand the ROS documentation of complex ROS packages for object recognition, text to speech, navigation and all the other areas where ROS developed code.

## Preliminary Installation

* __Ubuntu Installation__ :
For using ROS framework, Ubuntu is necessary. So, follow any of the four alternatives for setting up the linux environment:
(It's Preferable that you install Ubuntu 20.04)<br />
**Dual-boot**: Follow this [Tutorial](https://towardsdatascience.com/how-to-dual-boot-windows-10-and-linux-ubuntu-20-04-lts-in-a-few-hassle-free-steps-f0e465c3aafd) or this [Video Tutorial](https://www.youtube.com/watch?v=-iSAyiicyQY) to install ubuntu in your laptop.
<span style="color:red">[WARNING], Do at your own risk! We will be not responsible if you lose your data. __Follow instructions carefully and make backups before you start!__</span> <br />
*For absolute beginners, we recommend going for any one of the three alternatives mentioned below unless you're sure you want to dual-boot. Dual-booting can be a little daunting. And you can always opt for dual-booting once you're comfortable with linux.* <br />
**Virtual Machine** : You can install a virtual machine and install ubuntu on that. Follow this [Tutorial](https://www.youtube.com/watch?v=x5MhydijWmc) in that case. (Installation of the Virtual Machine is also included in the tutorial)<br />
**WSL** : Follow this [Tutorial](https://www.windowscentral.com/install-windows-subsystem-linux-windows-10) to install WSL on your windows system.<br />
**The Construct website** : You are also free to use [**theconstructsim**](https://www.theconstructsim.com/) without having to install anything. The Construct is an online platform that supports ROS development.

* __Get familiar with Linux__:
Here are a few resources that you can refer to in order to get familiar with Linux:
	* [Video-based Tutorial](https://www.youtube.com/watch?v=IVquJh3DXUA "Introduction to Linux and Basic Linux Commands for Beginners")
	* [Text-based Tutorial](https://ryanstutorials.net/linuxtutorial/ "Linux Tutorial")

* __Terminator installation for Ubuntu__:

	It's highly recommended to use Terminator instead of stock Terminal as you can have tabs or split windows into few terminals, which will be very useful throughout.

	To install **Terminator**, run the following command in the terminal:  
	```bash
	sudo apt-get install terminator
	```

* __ROS Installation/setup__:
	- For Ubuntu 20.04: [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu)
	- For Ubuntu 18.04: [ROS Melodic Morena](http://wiki.ros.org/melodic/Installation)    
Go to a particular link and put your first step in the world of ROS.

* __IDE Installation__:

	You are free to use a suitabe IDE to write code. The most commonly used IDE is **Visual Studio Code**. You can install it in your Ubuntu system and install **ROS VSCode Extention** in the VSCode application.


## **Getting started with the ROS:**

*Now that the installation is done, let’s dive into ROS!*

### **What is ROS?**

ROS is a software framework for writing robot software. The main aim of ROS is to reuse the robotic software across the globe. ROS consists of a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
The official definition on ROS wiki is:

*ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. ROS is similar in some respects to ‘robot frameworks, such as Player, YARP, Orocos, CARMEN, Orca, MOOS, and Microsoft Robotics Studio.*

### **Basics of ROS**

First of all, let us start with the basics of ROS.
It would be better if you **write the code on your own instead of copying and pasting it directly.** You will grasp the topics covered better when you try the implementation on your own. It's preferable if you use __Python__ instead of __C++__ as python syntax is easier and more readable.

To start off, these two tutorials will cover aspects such as **creating a ROS Workspace** and **navigating the ROS Filesystem** <br />
[Configuring your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) <br />
[Navigating the ROS filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)

#### **What is a package?** 

ROS uses **packages** to organize its programs. You can think of a package as **all the files that a specific ROS program contains**; all its CPP files, python files, configuration files, compilation files, launch files, and parameter files. All those files in the package are organized with the following structure:

* __launch__ folder: Contains launch files
* __src folder__: Source files (CPP, python)
* __CMakeLists.txt__: List of CMake rules for compilation
* __package.xml__: Package information and dependencies

To go to any ROS package, ROS gives you a command called `roscd`. Type:

`roscd <package_name>`

It will take you to the path where the package *package_name* is located. `roscd`  is a command which will get you to a ROS package location. `ls`is a command that lists the content of a folder.

* Every ROS program that you want to execute is organized in a package
* Every ROS program that you create will have to be organized in a package
* Packages are the main organizational system of ROS programs


#### **Create a new package**

Until now we’ve been checking the structure of an already-built package. But now, let’s create one ourselves. When we want to create packages, we need to work in a very specific ROS workspace, which is known as the catkin workspace. The **catkin workspace** is the directory in your hard disk where your own ROS packages must reside in order to be usable by ROS. Usually, the catkin workspace directory is called *catkin_ws* .

Usually, the *catkin_ws* is created in the home folder of your user account. The catkin workspace has been already created and initialized for you.

Go to the src folder inside *catkin_ws* :

```bash
cd ~/catkin_ws /src
```

The *src* directory is the folder that holds created packages. Those could be your own packages or packages that you copied from other sources e.g. A Github Repository.

In order for the ROS system to recognize the packages in your *catkin_ws*, it needs to be on the ROS file path. ROS file path is an Ubuntu environment variable that holds the paths to ROS packages. To add our *catkin_ws* to the ROS file path, follow the following instructions.

First, build (compile) your workspace. It’s OK to build the *catkin_ws*  even if it has no packages. After the build process, some new folders will appear inside your *catkin_ws* . One of the folders, called *catkin_ws* /devel contains a setup file that will be used to add the path of the *catkin_ws*  to the ROS file path. Build the *catkin_ws*  using the catkin build inside the *catkin_ws* :

```bash
cd ~/catkin_ws 	# Navigate to the catkin_ws
catkin build 	# Build
```
Now, let’s add the *catkin_ws*  path. Execute the following command while inside *catkin_ws* :

```bash
source devel/setup.bash
```
This adds the *catkin_ws*  path in the current terminal session but once you close the terminal window, it forgets it! So, you will have to do it again each time you open a terminal in order for ROS to recognize your workspace! Yeah, I know, that sucks! But no worries, there is a solution. You can automate the execution of the above command each time you open a terminal window. To do that, you want to add the above command to a special file called .bashrc that is located inside your home folder.

```bash
cd ~		# go to the home folder
nano .bashrc	# open the .bashrc file
```
Add the command `source ~/catkin_ws/devel/setup.bash` to the end of *.bashrc*.  
Then, hit <kbd>CTRL</kbd>+<kbd>X</kbd>, then, <kbd>Y</kbd>, to save the changes to the file.

Since your workspace has already been created, navigate to that workspace.
```bash
cd ~/catkin_ws/src
```
New packages are created using the `catkin_create_pkg`.The `catkin_create_pkg` requires you to provide a package name and a list of dependencies (optional) on which the package depends.
```bash
# This is an example, do not try to run this
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
Now let us create a package named 'beginner_tutorials'.
```bash
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```
Build the packages in the catkin workspace
```bash
cd ~/catkin_ws
catkin_make
```

To add the workspace to your ROS environment you need to source the generated setup file (if you have not automated the sourcing process):
```bash
. ~/catkin_ws/devel/setup.bash
```

For more clarity, go through these tutorials: <br />
[Creating a ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) <br />
[Building a ROS package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)

### Nodes
One of the primary purposes of ROS is to facilitate communication between the ROS nodes. Every program in ROS is called a **node**. Every independent task can be separated into nodes which communicate with each other through channels. These channels are also known as **topics**.

For example, one node can capture the images from a camera and send the images to another node for processing. After processing the image, the second node can send a control signal to a third node for controlling a robotic manipulator in response to the camera view.

The main mechanism used by ROS nodes to communicate is by sending and receiving **messages**. The **messages** are organized into specific categories called **topics**. Nodes may **publish** messages on a particular topic or **subscribe** to a topic to receive information.

**Important ROS Commands**:

`roscore`:The Main program to initiate ros. It sets up the basic architecture for the channels, allowing nodes to communicate.

`rosrun` is used to run a single ros program  (node).

`roslaunch` is used to automate launching multiple nodes at once.


Usage of rosrun command:
```bash
rosrun [package name] [node_name]
```


Let us run 'turtlesim_node' node from 'turtlesim' package using `rosrun`:

First, get the roscore running:
```bash
roscore
```
To run the 'turtlesim_node' node, run this **in a different terminal**:
```bash
rosrun turtlesim turtlesim_node
```
You'll see the new turtlesim window.

You are required go through this [Tutorial](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes) as well.


##### Using roslaunch to run multiple nodes at once

Usage:
```bash
roslaunch [package] [filename.launch]
```

To make a launch file, first go to the package where you plan on executing multiple nodes. Since we've already created a package 'beginners_tutorials', let us go there.
```bash
roscd beginner_tutorials
```
(If roscd says something similar to *roscd: No such package/stack 'beginner_tutorials'*, you'll need to source the environment setup file.  
```bash
cd ~/catkin_ws
source devel/setup.bash
roscd beginner_tutorials
```
Run the above commands in the terminal to do that)

Now let's make a launch directory:
```bash
mkdir launch
cd launch
```

(Let us use the 'turtlesim_node' node that we had run earlier to help us understand how to write a launch file)
Create a new launch file called turtlemimic.launch and paste the following :
```xml
<!--  we start the launch file with the launch tag, so that the file is identified as a launch file -->
<launch>
	
  <!-- Here we start two groups with a namespace tag of turtlesim1 and turtlesim2 with a turtlesim node with a name of sim. This allows us to start two simulators without having name conflicts -->
  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>
  
  <!-- Here we start the mimic node with the topics input and output renamed to turtlesim1 and turtlesim2. This renaming will cause turtlesim2 to mimic turtlesim1 -->
  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```

Now we roslaunch the ros file:
```bash
roslaunch beginner_tutorials turtlemimic.launch
```
We see two new turtlesim windows now

**In a new terminal**, send the rostopic command:
```bash
rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
We see that the two turtles are moving in a circle, even though the above command was passed to only turtlesim1. That is because the launch file has been written in such a way that the turtlesim2 mimics the turtlesim1.


#### Topics
You use a topic when you need to send a data stream. The data stream is unidirectional. Some nodes can **publish** on the topic, some nodes can **subscribe** to the topic. There is no response from a subscriber to a publisher, the data is only going one way.

A topic has a message type. All publishers and subscribers on this topic must use the message type associated with the topic.

You can create a publisher or subscriber in any ROS supported language you want, directly inside ROS nodes.

When a node wants to publish something, it will inform the ROS master. When another node wants to subscribe to a topic, it will ask the ROS master form where it can get the data.
(The rosmaster package implements the ROS Master. Most programs will not need to interact with this package directly. The rosmaster is run automatically whenever `roscore` is run and all communication with the Master happens over XMLRPC APIs.)

Finally, a node can contain many publishers and subscribers for many different topics.

Go through this [Turtorial](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics).

## Publisher Subscriber Interface

Message passing in ROS happens with the Publisher Subscriber Interface provided by ROS library functions.

Creating a publisher or subscriber node is just like creating any other node. <br />
Go to the package (where you want to create these nodes), make a new directory or folder (Let us follow the convention followed in the official ROS tutorials and call the new folder "scripts"). Add the publisher node python script and the subscriber node python script in this new folder scripts.


### Writing a simple Publisher Node

This is a basic publisher node python script (taken from the official ROS tutorials from the website, and comments are added to help you understand the working of each line):

```python
#!/usr/bin/env python
# license removed for brevity

#import the rospy package and the String message type 
import rospy
from std_msgs.msg import String

#function to publish messages at the rate of 10 per second
def talker():
    #define a topic to which the messages will be published
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    #initialize the Publisher node
    #Setting anonymous=True will append random integers at the end of the publisher node
    rospy.init_node('talker', anonymous=True)
    
    #publishes at a rate of 2 messages per second
    rate = rospy.Rate(10) # 10hz
    
    #Keep publishing the messages until the user interrupts
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
	
	#display the message on the terminal
        rospy.loginfo(hello_str)
	
	#publish the message to the topic
        pub.publish(hello_str)
	
	#rate.sleep() will help wait long enough to maintain the desired rate through the loop
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    #to capture the Interrupt signals that could be thrown by rate.sleep()
    except rospy.ROSInterruptException:
        pass

```
### Writing a simple Subscriber Node:

This is a basic subscriber node python script (taken from the official ROS tutorials from the website, and comments are added to help you understand the working of each line):

```python

#!/usr/bin/env python
import rospy
from std_msgs.msg import String

#Callback function to print the subscribed data on the terminal
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

#Subscriber node function which will subscribe the messages from the Topic
def listener():
    
    #initialize the subscriber node called 'listener'
    rospy.init_node('listener', anonymous=True)
    
    #This is to subscribe to the messages from the topic named 'chatter'
    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()   
```

We have to make the python scripts executable. The command for that:

```python
chmod +x <name of the python script>.py
```

Here, in place of "name of the python script", put the name of the python script that is added to the folder. You should run this command everytime you add a python script to the package.
	
And, add this snippet
```python	
catkin_install_python(PROGRAMS scripts/<name of the publisher python script>.py scripts/<name of the subscriber python script>.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)	
```
to your 'CMakeLists.txt'.
	
Finally, go to catkin_ws and build the package

```python	
cd ~/catkin_ws
catkin_make	
```

You are required to go through this [Tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) as well.

### Running the Publisher and Subscriber:

Make sure the roscore is running
```bash
roscore
```

Source your workspace's setup.sh file (in a new terminal)
```bash
 cd ~/catkin_ws
 source ./devel/setup.bash
```

To run the Publisher Node:
```bash
rosrun beginner_tutorials talker.py
```
('talker' is the name of the Publisher Node we created) <br />
You can see that 'hello world' is being printed. The Publisher Node is up and running!

To run the Subscriber Node (in a new terminal):
```bash
rosrun beginner_tutorials listener.py
```
('listener' is the name of the Subscriber Node we created) <br />
You can see that 'heard hello world' is being printed. The Subscriber Node is running as well.

Note that once you stop running the Publisher Node ( Press `Ctrl`+`C` while you're in the terminal that is running the Publisher Node), the Subscriber Node stops running as well. 

Go through this [Tutorial](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber) as well.



