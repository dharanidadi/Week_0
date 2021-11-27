# Week_0
* __Ubuntu Installation__ :
For using ROS framework Ubuntu is necessary:
(It's Preferable that you install Ubuntu 20.04)
Follow this [Tutorial](https://towardsdatascience.com/how-to-dual-boot-windows-10-and-linux-ubuntu-20-04-lts-in-a-few-hassle-free-steps-f0e465c3aafd) or this [Video Tutorial](https://www.youtube.com/watch?v=-iSAyiicyQY) to install ubuntu in your laptop.  
<span style="color:red">[WARNING], Do at your own risk! We will be not responsible if you lose your data. __Follow instructions carefully and make backups before you start!__</span>
If you're not sure if you want to dual boot, you can install a virtual machine and install ubuntu on that. Follow this [Tutorail](https://www.youtube.com/watch?v=x5MhydijWmc) in that case. (Installation of the Virtual Machine is also included in the tutorial)

* __Get familiar with Linux__:
Here are a few resources that you can refer to in order to get familiar with Linux:
	* [Video-based Tutorial](https://www.youtube.com/watch?v=IVquJh3DXUA "Introduction to Linux and Basic Linux Commands for Beginners")
	* [Text-based Tutorial](https://ryanstutorials.net/linuxtutorial/ "Linux Tutorial")

* __ROS Installation/setup__:
	- For Ubuntu 20.04: [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu)
	- For Ubuntu 18.04: [ROS Melodic Morena](http://wiki.ros.org/melodic/Installation)    
Go to a particular link and put your first step in the world of ROS.

### **Getting started with the ROS:**

*Now that the installation is done, let’s dive into ROS!*

#### **What is ROS?**

ROS is a software framework for writing robot software. The main aim of ROS is to reuse the robotic software across the globe. ROS consists of a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
The official definition on ROS wiki is:

*ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. ROS is similar in some respects to ‘robot frameworks, such as Player, YARP, Orocos, CARMEN, Orca, MOOS, and Microsoft Robotics Studio.*

#### **Basics of ROS**

First of all, let us start with the basics of ROS.
Go through the beginner level [Tutorials](http://wiki.ros.org/ROS/Tutorials). It will cover all the basics materials like how to create a package, what is a node and how to make it, what is a publisher and a subscriber? It will give you a detailed introduction to each and every thing. There are **20** parts to this tutorial, go through them all. Don't go to the intermediate level right now if you get done with the beginner level. We will give you an assignment based on these topics. **Write code on your own. Don't copy paste it directly.**  You will grasp the topics covered better when you try the implementation on your own. Assignment will be releasd soon. It's preferable if you use __Python__ instead of __C++__ as python syntax is easier and more readable and you will need it in future for sure, so, better learn it right now. These assignments will require knowledge of only basic python syntax.

Here, we are briefing about what is a package and how to create a package. This is also given in the tutorials, but we are telling this explicitly because it is the most fundamental thing that you have to do when you start with ROS.

##### **What is a package?** 

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


##### **Create a package**

Until now we’ve been checking the structure of an already-built package. But now, let’s create one ourselves. When we want to create packages, we need to work in a very specific ROS workspace, which is known as the catkin workspace. The **catkin workspace** is the directory in your hard disk where your own ROS packages must reside in order to be usable by ROS. Usually, the catkin workspace directory is called *catkin_ws* .

Usually, the *catkin_ws* is created in the home folder of your user account. The catkin workspace has been already created and initialized for you.

Go to the src folder inside *catkin_ws* :

```bash
cd ~/catkin_ws /src
```

The *src* directory is the folder that holds created packages. Those could be your own packages or packages that you copied from other sources e.g. A Github Repository.

In order for the ROS system to recognize the packages in your *catkin_ws*, it needs to be on the ROS file path. ROS file path is an Ubuntu environment variable that holds the paths to ROS packages. To add our *catkin_ws* to the ROS file path follow the following instructions.

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

Now, you can refer to the [tutorials](http://wiki.ros.org/ROS/Tutorials#Beginner_Level) on ROS wiki for further instructions.

Go through the official tutorials mentioned above and the explanations given below to maintain an intuitive understanding of the concepts.


#### Nodes
One of the primary purposes of ROS is to facilitate communication between the ROS nodes. Every program in ROS is called a **node**. Every independent task can be separated into nodes which communicate with each other through channels. These channels are also known as **topics**.

For example, one node can capture the images from a camera and send the images to another node for processing. After processing the image, the second node can send a control signal to a third node for controlling a robotic manipulator in response to the camera view.

The main mechanism used by ROS nodes to communicate is by sending and receiving **messages**. The **messages** are organized into specific categories called **topics**. Nodes may **publish** messages on a particular topic or **subscribe** to a topic to receive information.

Common ROS tools:

`roscore`:The Main program to initiate ros. It sets up the basic architecture for the channels, allowing nodes to communicate.

`rosrun` is used to run a single ros program  (node).

`roslaunch` is used to automate launching multiple nodes at once.

There are tools like `rostopic` and `rqt_graph` which can be used to visualize the nodes’ communication in the current channels and their message transactions.

#### Topics
You use a topic when you need to send a data stream. The data stream is unidirectional. Some nodes can publish on the topic, some nodes can subscribe to the topic. There is no response from a subscriber to a publisher, the data is only going one way.

A topic has a message type. All publishers and subscribers on this topic must use the message type associated with the topic.

You can create a publisher or subscriber in any ROS supported language you want, directly inside ROS nodes.

When a node wants to publish something, it will inform the ROS master. When another node wants to subscribe to a topic, it will ask the ROS master form where it can get the data.
(The rosmaster package implements the ROS Master. Most programs will not need to interact with this package directly. The rosmaster is run automatically whenever `roscore` is run and all communication with the Master happens over XMLRPC APIs.)

Finally, a node can contain many publishers and subscribers for many different topics.

#### Services

A ROS service is a client/server system

Services are another way to pass data between nodes in ROS. Services are just synchronous remote procedure calls, they allow one node to call a function that executes in another node. Service calls are well suited to things that you only need to do occasionally and that take a bounded amount of time to complete.

A service is defined by a name, and a pair of messages. One message is the request, one message is the response. You must respect the format of the data on both sides of the communication.

You can directly create service clients and servers inside ROS nodes, using for example, the roscpp library for c++ and the rospy library for Python.

Finally, a service server can only exist once, but can have many clients. And basically, the service will be created when you create the server.

(Topics will be used for unidirectional data streams, and services will be used when you need a client/server architecture.)
![](Nodes-TopicandService.gif)

#### Parameters

A centralized parameter server keeps track of a collection of values-things like integers, floating point numbers, strings, or other data-each identified by a short string name.

A ROS parameter is basically just one of the shared variable stored in the parameter server.
Ex:
![](parameters.jpg)

In this example, we have 4 nodes included in 3 different packages. As you can see, any node from any package can get access to the ROS parameter server. The only condition is that the nodes should be on the same environment as the ROS master.

When node A is started, it can add a new parameter into the parameter server. If node B starts after node A, node B will have access to this new parameter.






