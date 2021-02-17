package uk.ac.liverpool.lrcfmd.kmriiwa.nodes;


import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
 


public class PublicationNode extends AbstractNodeMain {
	
	// ROS publishers for KMRIIWA platform
	// Joint state publisher
	private Publisher<sensor_msgs.JointState> jointStatePublisher;
	// IIWA Joint position publisher
	private Publisher<iiwa_msgs.JointPosition> jointPositionPublisher;
	// End effector pose publisher
	private Publisher<iiwa_msgs.CartesianPose> cartesianPosePublisher;
	// DestinationReachedPublisher flag publisher for the arm
	private Publisher<std_msgs.String> armDestinationReachedPublisher;
	// DestinationReachedPublisher flag publisher for the grippper
	private Publisher<std_msgs.String> gripperDestinationReachedPublisher;
	
	private Publisher<sensor_msgs.LaserScan> laserB1ScanPublisher;
	private Publisher<sensor_msgs.LaserScan> laserB4ScanPublisher;
	private Publisher<geometry_msgs.Pose> basePosePublisher;
	// Robot name used to build ROS topics
	private String robotName = "kmriiwa";
	// true if node is connected to ROS master
	private boolean connectedToMaster = false;
	
	private ConnectedNode node = null;
	
	public PublicationNode(String robotName) 
	{
		this.robotName = robotName;
	}
	

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(robotName + "/publisher");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode)
	{
		node = connectedNode;
		
		jointStatePublisher = node.newPublisher(robotName + "/arm/joint_states", sensor_msgs.JointState._TYPE);
		cartesianPosePublisher = node.newPublisher(robotName + "/arm/state/CartesianPose", iiwa_msgs.CartesianPose._TYPE);
		jointPositionPublisher = node.newPublisher(robotName + "/arm/state/JointPosition", iiwa_msgs.JointPosition._TYPE);
		laserB1ScanPublisher = node.newPublisher(robotName + "/base/state/LaserB1Scan", sensor_msgs.LaserScan._TYPE);
		laserB4ScanPublisher = node.newPublisher(robotName + "/base/state/LaserB4Scan", sensor_msgs.LaserScan._TYPE);
		basePosePublisher = node.newPublisher(robotName + "/base/state/Pose", geometry_msgs.Pose._TYPE);
		armDestinationReachedPublisher = node.newPublisher(robotName + "/arm/state/DestinationReached", std_msgs.String._TYPE);
		gripperDestinationReachedPublisher = node.newPublisher(robotName + "/gripper/state/DestinationReached", std_msgs.String._TYPE);
		
		connectedToMaster = true;
	}
	
	public <T extends org.ros.internal.message.Message> void publish(T msg) throws InterruptedException
	{
		if (msg instanceof sensor_msgs.JointState)
		{
			jointStatePublisher.publish((sensor_msgs.JointState) msg);
		}
		else if (msg instanceof iiwa_msgs.JointPosition)
		{
			jointPositionPublisher.publish((iiwa_msgs.JointPosition) msg);
		}
		else if (msg instanceof iiwa_msgs.CartesianPose)
		{
			cartesianPosePublisher.publish((iiwa_msgs.CartesianPose) msg);
		}
		else if (msg instanceof sensor_msgs.LaserScan)
		{
			if (((sensor_msgs.LaserScan) msg).getHeader().getFrameId().matches("laser_B1_link"))
				laserB1ScanPublisher.publish((sensor_msgs.LaserScan) msg);
			else
			{
				laserB4ScanPublisher.publish((sensor_msgs.LaserScan) msg);
			}
		}
		else if (msg instanceof geometry_msgs.Pose)
		{
			basePosePublisher.publish((geometry_msgs.Pose) msg);
		}
		else
		{
			System.out.println("Couldn't send unknown message type");
		}
	}
	
	public synchronized void publishArmDestinationReached() 
	{
    	std_msgs.String reachedMsg = node.getTopicMessageFactory().newFromType(std_msgs.String._TYPE);
    	reachedMsg.setData("done");
    	armDestinationReachedPublisher.publish(reachedMsg);
    }
	
	public synchronized void publishGripperDestinationReached() 
	{
    	std_msgs.String reachedMsg = node.getTopicMessageFactory().newFromType(std_msgs.String._TYPE);
    	reachedMsg.setData("done");
    	gripperDestinationReachedPublisher.publish(reachedMsg);
    }
	
	public boolean isConnectedToMaster()
	{
		return connectedToMaster;
	}
	

}
