package uk.ac.liverpool.lrcfmd.kmriiwa.utility;

import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.PublicationNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRMsgGenerator;

public class PublisherTask implements Runnable {
	
	private PublicationNode publisher = null;
	private LBRMsgGenerator lbrMsgGenerator = null;
	
	public PublisherTask(PublicationNode publisher, LBRMsgGenerator lbrMsgGenerator)
	{
		this.publisher = publisher;
		this.lbrMsgGenerator = lbrMsgGenerator;
	}

	@Override
	public void run() {
		try
		{
				sensor_msgs.JointState jsMsg = lbrMsgGenerator.getCurrentJointState();
				publisher.publish(jsMsg);
				iiwa_msgs.CartesianPose cpMsg = lbrMsgGenerator.getCurrentCartesianPose();
				publisher.publish(cpMsg);
				iiwa_msgs.JointPosition jpMsg = lbrMsgGenerator.getCurrentJointPosition();
				publisher.publish(jpMsg);
		}
		catch (Exception e)
		{
			System.out.println(e.getMessage());
			e.printStackTrace();
			throw new RuntimeException("Publisher couldn't publish messages");
		}

	}
}
