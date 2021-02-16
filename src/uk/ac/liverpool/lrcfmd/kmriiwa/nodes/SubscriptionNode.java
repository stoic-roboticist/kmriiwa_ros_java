package uk.ac.liverpool.lrcfmd.kmriiwa.nodes;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;


public class SubscriptionNode extends AbstractNodeMain
{
	
	private ConnectedNode node = null;
	private String robotName = "kmriiwa";
	// true if node is connected to ROS master
	private boolean connectedToMaster = false;
	
	private Subscriber<iiwa_msgs.JointPosition> jointPositionSubscriber;
	private Subscriber<std_msgs.Bool> openGripperSubscriber;
	
	private iiwa_msgs.JointPosition jp;
	private std_msgs.Bool openGrp;
	
	private Boolean new_jp = new Boolean(false);
	private Boolean new_openGrp = new Boolean(false);
	
	public SubscriptionNode(String robotName)
	{
		this.robotName = robotName;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(robotName + "/subscriber");
	}
	
	@Override
	public void onStart(ConnectedNode connectedNode)
	{
		node = connectedNode;
		jp = node.getTopicMessageFactory().newFromType(iiwa_msgs.JointPosition._TYPE);
		openGrp = node.getTopicMessageFactory().newFromType(std_msgs.Bool._TYPE);
		
		jointPositionSubscriber = node.newSubscriber(robotName + "/command/JointPosition", iiwa_msgs.JointPosition._TYPE);
		jointPositionSubscriber.addMessageListener( new MessageListener<iiwa_msgs.JointPosition>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointPosition position)
			{
				synchronized(new_jp)
				{
					jp = position;
					new_jp = true;
				}
			}
		});
		
		openGripperSubscriber = node.newSubscriber(robotName + "/command/openGripper", std_msgs.Bool._TYPE);
		openGripperSubscriber.addMessageListener( new MessageListener<std_msgs.Bool>() {
			@Override
			public void onNewMessage(std_msgs.Bool cmd)
			{
				synchronized(new_openGrp)
				{
					openGrp = cmd;
					new_openGrp = true;
				}
			}
		});
		
		connectedToMaster = true;
	}
	
	public iiwa_msgs.JointPosition getJointPositionTarget()
	{
		synchronized (new_jp) 
		{
			if (new_jp) 
			{
				new_jp = false;
				return jp;
			}
			else 
			{
				return null;
			}
	    }
	}
	
	public std_msgs.Bool getOpenGripperCmd()
	{
		synchronized (new_openGrp) 
		{
			if (new_openGrp) 
			{
				new_openGrp = false;
				return openGrp;
			}
			else 
			{
				return null;
			}
	    }
	}
	
	public boolean isConnectedToMaster()
	{
		return connectedToMaster;
	}
}