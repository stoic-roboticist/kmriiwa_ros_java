package uk.ac.liverpool.lrcfmd.kmriiwa.nodes;


import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.rosjava.tf.pubsub.TransformBroadcaster;

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
	// KMR base state publisher
	private Publisher<kmriiwa_msgs.KMRStatus> KMRStatusPublisher;
	// LBR state publisher
	private Publisher<kmriiwa_msgs.LBRStatus> LBRStatusPublisher;
	
	private Publisher<sensor_msgs.LaserScan> laserB1ScanPublisher;
	private Publisher<sensor_msgs.LaserScan> laserB4ScanPublisher;
	private Publisher<nav_msgs.Odometry> odometryPublisher;
	private TransformBroadcaster tfPublisher;
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
		odometryPublisher = node.newPublisher(robotName + "/base/state/odom", nav_msgs.Odometry._TYPE);
		KMRStatusPublisher = node.newPublisher(robotName + "/base/state/RobotStatus", kmriiwa_msgs.KMRStatus._TYPE);
		armDestinationReachedPublisher = node.newPublisher(robotName + "/arm/state/DestinationReached", std_msgs.String._TYPE);
		gripperDestinationReachedPublisher = node.newPublisher(robotName + "/gripper/state/DestinationReached", std_msgs.String._TYPE);
		LBRStatusPublisher = node.newPublisher(robotName + "/arm/state/RobotStatus", kmriiwa_msgs.LBRStatus._TYPE);
		tfPublisher = new TransformBroadcaster(node);
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
		else if (msg instanceof nav_msgs.Odometry)
		{
			odometryPublisher.publish((nav_msgs.Odometry) msg);
		}
		else if (msg instanceof kmriiwa_msgs.KMRStatus)
		{
			KMRStatusPublisher.publish((kmriiwa_msgs.KMRStatus) msg);
		}
		else if (msg instanceof kmriiwa_msgs.LBRStatus)
		{
			LBRStatusPublisher.publish((kmriiwa_msgs.LBRStatus) msg);
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
	
	public void publishTransform(String parentFrame, String childFrame, long time,geometry_msgs.Pose pose)
	{
		tfPublisher.sendTransform(parentFrame, childFrame, 
								  time, 
								  pose.getPosition().getX(), 
								  pose.getPosition().getY(),
								  pose.getPosition().getZ(),
								  pose.getOrientation().getX(),
								  pose.getOrientation().getY(),
								  pose.getOrientation().getZ(),
								  pose.getOrientation().getW());
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
