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
	
	private Subscriber<kmriiwa_msgs.JointPosition> jointPositionSubscriber;
	private Subscriber<geometry_msgs.Twist> baseTwistSubscriber;
	
	private kmriiwa_msgs.JointPosition jp;
	private geometry_msgs.Twist baseTwist;
	
	private Boolean new_jp = new Boolean(false);
	private Boolean new_baseTwist = new Boolean(false);
	
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
		jp = node.getTopicMessageFactory().newFromType(kmriiwa_msgs.JointPosition._TYPE);
		
		jointPositionSubscriber = node.newSubscriber(robotName + "/arm/command/JointPosition", kmriiwa_msgs.JointPosition._TYPE);
		jointPositionSubscriber.addMessageListener( new MessageListener<kmriiwa_msgs.JointPosition>() {
			@Override
			public void onNewMessage(kmriiwa_msgs.JointPosition position)
			{
				synchronized(new_jp)
				{
					jp = position;
					new_jp = true;
				}
			}
		});
		
		baseTwistSubscriber = node.newSubscriber(robotName + "/base/command/cmd_vel", geometry_msgs.Twist._TYPE);
		baseTwistSubscriber.addMessageListener( new MessageListener<geometry_msgs.Twist>() {
			@Override
			public void onNewMessage(geometry_msgs.Twist twist)
			{
				synchronized(new_baseTwist)
				{
					baseTwist = twist;
					new_baseTwist = true;
				}
			}
		});
		
		connectedToMaster = true;
	}
	
	public kmriiwa_msgs.JointPosition getJointPositionTarget()
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
	
	
	public geometry_msgs.Twist getBaseTwistTarget()
	{
		synchronized (new_baseTwist) 
		{
			if (new_baseTwist) 
			{
				new_baseTwist = false;
				return baseTwist;
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