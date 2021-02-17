package uk.ac.liverpool.lrcfmd.kmriiwa.utility;

import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.PublicationNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.KMRMsgGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.KMRMsgGenerator.LaserScanner;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRMsgGenerator;

public class PublisherTask implements Runnable {
	
	private PublicationNode publisher = null;
	private LBRMsgGenerator lbrMsgGenerator = null;
	private KMRMsgGenerator kmrMsgGenerator = null;
	
	public PublisherTask(PublicationNode publisher, LBRMsgGenerator lbrMsgGenerator, KMRMsgGenerator kmrMsgGenerator)
	{
		this.publisher = publisher;
		this.lbrMsgGenerator = lbrMsgGenerator;
		this.kmrMsgGenerator = kmrMsgGenerator;
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
				sensor_msgs.LaserScan lsb1Msg = kmrMsgGenerator.getLaserScan(LaserScanner.LASER_B1);
				publisher.publish(lsb1Msg);
				sensor_msgs.LaserScan lsb4Msg = kmrMsgGenerator.getLaserScan(LaserScanner.LASER_B4);
				publisher.publish(lsb4Msg);
				geometry_msgs.Pose bpMsg = kmrMsgGenerator.getBasePose();
				publisher.publish(bpMsg);
		}
		catch (Exception e)
		{
			System.out.println(e.getMessage());
			e.printStackTrace();
			throw new RuntimeException("Publisher couldn't publish messages");
		}

	}
}
