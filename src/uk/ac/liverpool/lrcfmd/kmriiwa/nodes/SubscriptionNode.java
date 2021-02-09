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
	
	private Subscriber<iiwa_msgs.JointPosition> jointPositionSubscriber;
	
	private iiwa_msgs.JointPosition jp;
	
	private Boolean new_jp = new Boolean(false);
	
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
}