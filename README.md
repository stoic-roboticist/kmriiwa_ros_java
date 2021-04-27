# *KMRIIWA_ROS_JAVA package*
A ROS driver for the KMRIIWA robot developed in pure JAVA using rosjava library. This driver is designed to be used with [kmriiwa_ros_stack repository](https://github.com/stoic-roboticist/kmriiwa_ros_stack) repo that utilises this driver's interfaces to provide a complete ROS package for the KMRIIWA robot. 

**This driver was tested on Sunrise 1.16**

**To control the robot via ROS use [kmriiwa_ros_stack repository](https://github.com/stoic-roboticist/kmriiwa_ros_stack)**

## Description
This driver launches a ROS node on the robot controller that exposes a number of ROS interfaces. These interfaces allow to command the robot and read its state. These interfaces are gouped under the robot namespace and are further divided according to the robot components, which include the LBR-IIWA14 arm and KMP200 Base. In addition optional interfaces can be exposed for a tool attahced to the robot arm. Below is a description of the exposed interfaces:
- **LBR-IIWA14 arm state interfaces**
  - ***<robot_ns>/arm/joint_states***: This topic publishes the arm's joint states using sensor_msgs/JointState messages.
  - ***<robot_ns>/arm/state/RobotStatus***: This topic publishes the arm's state in terms of flags that indicate if its joints are mastered, referenced, enabled for motion and not in emergency stop. This topic publishes this information using kmriiwa_msgs/LBRStatus message.
- **LBR-IIWA14 arm command interfaces**
  - ***<robot_ns>/arm/command/JointPosition***: This topic accepts a message of type kmriiwa_msgs/JointPosition and commands the arm to move the specified joint angles. A message is sent on the *<robot_ns>/arm/state/JointPositionReached* with the outcome of the motion. A message (of type std_msgs/String) with string "success" is sent if the motion is successful, otherwise a message with string "fail" is sent.
  - ***<robot_ns>/arm/manipulator_controller/follow_joint_trajectory/<action_server_topics>***: This action server accepts *control_msgs/FollowJointTrajectory* goals and commands the robot to perform the received trajectories. Currently, it only executes the trajectory joint positions at constant speed and doesn't utilise the joint velocities, accelerations and efforts. Moreover, no feedback is sent. When the action is complete a result is sent back to the client with the execution outcome. This allows the driver to work with moveit MoveIt motion planning framework.
- **KMP200 mobile base state interfaces**
  -  ***<robot_ns>/base/state/LaserB1Scan***: This topic publishes the readings of SICK300 laser scanner sensor situated on the front side of the robot using sensor_msgs/LaserScan messages.
  -  ***<robot_ns>/base/state/LaserB4Scan***: This topic publishes the readings of SICK300 laser scanner sensor situated on the back side of the robot using sensor_msgs/LaserScan messages.
  -  ***<robot_ns>/base/state/odom***: This topic publishes the odometry readings of the base using nav_msgs/Odometry messages. Currently, these messages do not include any covariance information since the manual doesn't provide any.
  -  ***<robot_ns>/base/state/RobotStatus***: This topic publishes the base's state. This includes the battery charge state percentage, the robot motion and emergency state and the laser scanners warning and safety fields bits. This topic publishes this information using kmriiwa_msgs/KMRStatus message.
- **KMP200 mobile command state interfaces**
  -  ***<robot_ns>/base/command/cmd_vel***: This topic allows to jog the robot using geometry_msgs/Twist messages. As a result this allows the robot to work with ROS navigation stack in addition to all the mobile base state topics.

In addition to these interfaces, this driver also provides an abstract class ***ToolNode*** that can be used to incorporate any tool/gripper attached to the robot arm. It represent a ROS node that would expose a tool state and command interfaces. To achieve that, a class needs to be created that extends the abstract class and provide implementation for its method. 
TODO add an example project.  

## Setup
The KUKA KMRIIWA robot is programmed using the KUKA's Sunrise Workbench and their Java APIs.
A Sunrise project, containing one or more Robotic Application can be synchronized with the robot controller and executed from the SmartPad.
This package represent such a robotic application that can be synched with the robot controller and run from the SmartPad. To install this robotic application on the robot controller complete the following steps:
### **Setup the Sunrise Workbench Project**
  1. In the **Sunrise Workbench**, create a new sunrise project. Alternatively, you can copy a pre-existing prject from your workspace or load it from the robot controller.
  2. Open the **StationSetup.cat** 
  3. Ensure that the software page have more or less the following components:
  ![sunrise_software_tab](https://user-images.githubusercontent.com/13589969/116313878-45a84f00-a7a6-11eb-8490-48b602fcd85e.png)

### **Clone and setup the robotic application**
  1. Clone this repository to a location of your preference. We'll refer to this location as `KMRIIWA_ROS_JAVA_DIR`.
  2. To make a link to the source code such that it can track git changes <details><summary>Click the arrow and follow these steps</summary>
     1. Right click on your `src` folder inside the newly created project and select New > Other
     2. Under *General* select *Folder* and click next
     3. In the next window click *Advanced >>* then tick *Link to alternate location (Linked Folder)*
     4. Click *Browse* to select the folder `KMRIIWA_ROS_JAVA_DIR/src/uk` and then click *Finish*. This should add the repo's java packages to the project<br></details>
  
        Alternatively, just copy the `KMRIIWA_ROS_JAVA_DIR/src/uk` folder into your project *src* folder. This way the used code won't track git changes and you would need to be copy and paste everytime a new code is developed
  
     5. Copy the `ROSJavaLib` into your sunrise project (TODO link folder)
      
       1. Inside the linked/copied `ROSJavaLib` folder select all the files inside, right click and choose Build Path -> Add to Build Path...
       2. copy the files 'log4j_\*.jar, com.kuka.nav.provider_\*.jar and ' com.kuka.nav.robot.fdi.api_\*.jar' from the `plugins` folder in your Sunrise Workbench installation directory. The asterisk is used as a wildcard operator where these file names contain version number and other descriptors
       3. Inside the Sunrise Workbench, paste these files into the `KUKAJavaLib` folder in your project. Select these files, right click and choose Build Path -> Add to Build Path...
   
   ### **Setup the ProcessData Configuration**
   In your Sunrise Project, open the file `src/RoboticsAPI.data.xml` and modify its content to

```xml
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<RoboticsAPIData version="3">
    <world>
        <gravitation x="0.0" y="0.0" z="9.81"/>
    </world>
    <processDataContainer>
        <processData dataType="java.lang.String" defaultValue="kmriiwa" displayName="Robot Name" editableOnHmi="true" id="robot_name" value="kmriiwa"/>
        <processData dataType="java.lang.String" defaultValue="172.31.1.77" displayName="ROS Master IP" editableOnHmi="true" id="master_ip" value="172.31.1.77"/>
        <processData dataType="java.lang.String" defaultValue="11311" displayName="ROS Master Port" editableOnHmi="false" id="master_port" value="11311"/>
        <processData dataType="java.lang.String" defaultValue="172.31.1.10" displayName="Robot IP" editableOnHmi="false" id="robot_ip" value="172.31.1.10" visibleOnHmi="false"/>
        <processData dataType="java.lang.Boolean" defaultValue="false" displayName="Enable NTP" editableOnHmi="true" id="ntp" value="false"/>
    </processDataContainer>
</RoboticsAPIData>
```
  This enables you to dynamically reconfigure the robotic application parameters without having to resynchronize the project everytime. These options will be available on the Process Data window of the SmartPad.

Note that your RoboticsAPI.data.xml file might contain other parameters. In that case just copy the items above inside the processDataContainer field into the respective field on your file.

**Your sunrise project should be error-free and ready to be installed and synchronized with the robot controller.**


### **Setup ROS Master**
1. In order to use this driver, a roscore is required to be running on a machine connected to the KMRIIWA wireless network. To achieve that, setup the network interface on the machine designated to be the ROS master to have a static IP address and be in the same subnet. (e.g. 172.31.1.77). To check that your setup is working, ping the robot controller from your ROS master machine.

2. Change the parameters previously set in the ProcessData configuration step, to match the ROS master IP and port values set here. This can be done using the Process Data window of the SmartPad or editing the RoboticsAPI.data.xml default values.

### **Setup NTP server (Optional if no time stamp issues)**
If the clocks of the robot controller and the roscore machine are out of sync, problems will arise when publishing laser scans and odometry information. This will be due to receiving tf information in the past or future, which gets discarded by most ROS planning packages. 

If no tf problems are encountred with the default time provider when running the driver, this step can be skipped.

Alternatively, in case such problems arise, an NTP server would need to be running on the ROS master machine. Follow these steps to use an NTP server:

1. To setup NTP server on the ROS master machine follow the steps in the [link](https://linuxconfig.org/ubuntu-20-04-ntp-server). Because NTP server require internet access to sync clocks, make sure the ROS machine is connected to the internet when running the roscore.

2. To enable the application running on the robot to synchronize with the NTP server, set the parameter *Enable NTP* to true in the ProcessData configuration.


   
## Usage
After the setup is done there should be a Robotic Application installed on the robot names *ROSKmriiwaController*.
As mentioned before, this application requires having a ROS Master running on the ROS machine connected to the robot wireless network. That basically means running a roscore or call a launch file on a terminal. If you start *ROSKmriiwaController* without a ROS Master being online, it will wait for 2 minute until one is started.
It should not matter which one you start first, ROSKmriiwaController or roscore.

Once you start both, the robot should connect to the ROS Master and some notification will appear on the SmartPad. Everything should be set now to use the [kmriiwa_ros_stack repository](https://github.com/stoic-roboticist/kmriiwa_ros_stack). 

You can check if everything is okay by running the command rostopic list on any ROS machine connected to the master. This should list all the topics detailed in the Description section.

## Notes
- This package do not modify the robot safety configuration and do not violate any of its rules. The safety controller is running all the time and will stop the application if any error or safety violation arises.
- This driver doesn't support any low level ros_control interfaces because we don't have access to KUKA's official SmartServoing and FRI packages that would allow us to use these capabilities.  
  
