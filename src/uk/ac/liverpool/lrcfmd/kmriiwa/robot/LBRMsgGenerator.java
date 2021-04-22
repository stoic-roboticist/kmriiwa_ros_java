package uk.ac.liverpool.lrcfmd.kmriiwa.robot;

import java.util.Arrays;

import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.SafetyStopType;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;


import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.time.TimeProvider;

public class LBRMsgGenerator {
	
	private LBR robot;
	
	// Needed to create ROS messages
	private NodeConfiguration nodeConf = NodeConfiguration.newPrivate();
	private MessageFactory messageFactory = nodeConf.getTopicMessageFactory();
	private TimeProvider time;
	
	private String[] joint_names;
	
	
	/**
	 * 
	 * @param lbr
	 * @param name
	 * @param timeProvider
	 */
	public LBRMsgGenerator(LBR lbr, TimeProvider timeProvider)
	{
		this.robot = lbr;
		this.time = timeProvider;
		
		joint_names = new String[] { "kmriiwa_joint_1", "kmriiwa_joint_2", "kmriiwa_joint_3", 
				"kmriiwa_joint_4", "kmriiwa_joint_5", "kmriiwa_joint_6",
		        "kmriiwa_joint_7" };
	}

	public sensor_msgs.JointState getCurrentJointState()
	{
		sensor_msgs.JointState msg = messageFactory.newFromType(sensor_msgs.JointState._TYPE);

		msg.getHeader().setStamp(time.getCurrentTime());
		msg.setName(Arrays.asList(joint_names));
		msg.setPosition(robot.getCurrentJointPosition().getInternalArray());
		msg.setEffort(robot.getMeasuredTorque().getTorqueValues());
		
		return msg;
	}
	
	public kmriiwa_msgs.LBRStatus getLBRStatus()
	{
		
		kmriiwa_msgs.LBRStatus msg = messageFactory.newFromType(kmriiwa_msgs.LBRStatus._TYPE);
		
		msg.getHeader().setStamp(time.getCurrentTime());
		msg.setMotionEnabled(robot.isMotionEnabled());
		msg.setAxesMastered(robot.isMastered());
		msg.setAxesGmsReferenced(robot.getSafetyState().areAllAxesGMSReferenced());
		msg.setAxesPositionReferenced(robot.getSafetyState().areAllAxesPositionReferenced());
		msg.setSafetyStateEnabled(robot.getSafetyState().getSafetyStopSignal().compareTo(SafetyStopType.NOSTOP) != 0);
		
		return msg;
	}
}
