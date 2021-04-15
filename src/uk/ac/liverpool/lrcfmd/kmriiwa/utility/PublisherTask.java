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
				iiwa_msgs.CartesianPose cpMsg = lbrMsgGenerator.getCurrentCartesianPose();
				publisher.publish(cpMsg);
				iiwa_msgs.JointPosition jpMsg = lbrMsgGenerator.getCurrentJointPosition();
				publisher.publish(jpMsg);
				sensor_msgs.LaserScan lsb1Msg = kmrMsgGenerator.getLaserScan(LaserScanner.LASER_B1);
				publisher.publish(lsb1Msg);
				sensor_msgs.LaserScan lsb4Msg = kmrMsgGenerator.getLaserScan(LaserScanner.LASER_B4);
				publisher.publish(lsb4Msg);
				nav_msgs.Odometry boMsg = kmrMsgGenerator.getBaseOdometry();
				publisher.publish(boMsg);
				publisher.publishTransform(boMsg.getHeader().getFrameId(),
										   boMsg.getChildFrameId(),
										   boMsg.getHeader().getStamp().totalNsecs(),
										   boMsg.getPose().getPose());
				sensor_msgs.JointState jsMsg = lbrMsgGenerator.getCurrentJointState();
				publisher.publish(jsMsg);
				kmriiwa_msgs.KMRStatus ksMsg = kmrMsgGenerator.getKMRStatus();
				publisher.publish(ksMsg);
				kmriiwa_msgs.LBRStatus lsMsg = lbrMsgGenerator.getLBRStatus();
				publisher.publish(lsMsg);
		}
		catch (Exception e)
		{
			System.out.println(e.getMessage());
			e.printStackTrace();
			throw new RuntimeException("Publisher couldn't publish messages");
		}

	}
}
