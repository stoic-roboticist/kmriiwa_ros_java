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
	// DestinationReachedPublisher flag publisher
	private Publisher<std_msgs.Time> destinationReachedPublisher;
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
		
		jointStatePublisher = node.newPublisher(robotName + "/joint_states", sensor_msgs.JointState._TYPE);
		cartesianPosePublisher = node.newPublisher(robotName + "/state/CartesianPose", iiwa_msgs.CartesianPose._TYPE);
		jointPositionPublisher = node.newPublisher(robotName + "/state/JointPosition", iiwa_msgs.JointPosition._TYPE);
		destinationReachedPublisher = node.newPublisher(robotName + "/state/DestinationReached", std_msgs.Time._TYPE);
		
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
		else
		{
			System.out.println("Couldn't send unknown message type");
		}
	}
	
	public synchronized void publishDestinationReached() 
	{
    	std_msgs.Time currentTime = node.getTopicMessageFactory().newFromType(std_msgs.Time._TYPE);
    	currentTime.setData(node.getCurrentTime());
        destinationReachedPublisher.publish(currentTime);
    }
	
	public boolean isConnectedToMaster()
	{
		return connectedToMaster;
	}
	

}
