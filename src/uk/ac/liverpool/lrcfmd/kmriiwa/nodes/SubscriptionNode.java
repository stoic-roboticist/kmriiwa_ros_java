package uk.ac.liverpool.lrcfmd.kmriiwa.nodes;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;


public class SubscriptionNode extends AbstractNodeMain
{
	
	private ConnectedNode node = null;
	private String robotName = "kmriiwa_1";
	// true if node is connected to ROS master
	private boolean connectedToMaster = false;
	
	private Subscriber<iiwa_msgs.JointPosition> jointPositionSubscriber;
	private Subscriber<std_msgs.Bool> openGripperSubscriber;
	private Subscriber<geometry_msgs.Twist> baseTwistSubscriber;
	
	private iiwa_msgs.JointPosition jp;
	private std_msgs.Bool openGrp;
	private geometry_msgs.Twist baseTwist;
	
	private Boolean new_jp = new Boolean(false);
	private Boolean new_openGrp = new Boolean(false);
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
		jp = node.getTopicMessageFactory().newFromType(iiwa_msgs.JointPosition._TYPE);
		openGrp = node.getTopicMessageFactory().newFromType(std_msgs.Bool._TYPE);
		
		jointPositionSubscriber = node.newSubscriber(robotName + "/arm/command/JointPosition", iiwa_msgs.JointPosition._TYPE);
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
		
		openGripperSubscriber = node.newSubscriber(robotName + "/gripper/command/OpenGripper", std_msgs.Bool._TYPE);
		openGripperSubscriber.addMessageListener( new MessageListener<std_msgs.Bool>() {
			@Override
			public void onNewMessage(std_msgs.Bool cmd)
			{
				System.out.println("In sub node gripper msg:" + cmd.getData());
				synchronized(new_openGrp)
				{
					openGrp = cmd;
					new_openGrp = true;
				}
			}
		});
		
		baseTwistSubscriber = node.newSubscriber(robotName + "/base/command/Twist", geometry_msgs.Twist._TYPE);
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