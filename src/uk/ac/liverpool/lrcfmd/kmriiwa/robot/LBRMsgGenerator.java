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

import uk.ac.liverpool.lrcfmd.kmriiwa.utility.Logger;

public class LBRMsgGenerator {
	
	private LBR robot;
	
	// Needed to create ROS messages
	private NodeConfiguration nodeConf = NodeConfiguration.newPrivate();
	private MessageFactory messageFactory = nodeConf.getTopicMessageFactory();
	private TimeProvider time;
	
	private String robotName;
	private String[] joint_names;
	
	
	/**
	 * 
	 * @param lbr
	 * @param name
	 * @param timeProvider
	 */
	public LBRMsgGenerator(LBR lbr, String robotName, TimeProvider timeProvider)
	{
		this.robot = lbr;
		this.robotName = robotName;
		this.time = timeProvider;
		
		joint_names = new String[] { robotName + "_joint_1", robotName + "_joint_2", robotName + "_joint_3", 
				robotName + "_joint_4", robotName + "_joint_5", robotName + "_joint_6",
				robotName + "_joint_7" };
	}

	public sensor_msgs.JointState getCurrentJointState()
	{
		sensor_msgs.JointState msg = messageFactory.newFromType(sensor_msgs.JointState._TYPE);
		try
		{
			msg.getHeader().setStamp(time.getCurrentTime());
			msg.setName(Arrays.asList(joint_names));
			msg.setPosition(robot.getCurrentJointPosition().getInternalArray());
			msg.setEffort(robot.getMeasuredTorque().getTorqueValues());
		}
		catch (NullPointerException e)
		{
			Logger.warn("No joint state data available");
			Logger.warn("Empty JointState message is generated");
		}
		return msg;
	}
	
	public kmriiwa_msgs.LBRStatus getLBRStatus()
	{
		
		kmriiwa_msgs.LBRStatus msg = messageFactory.newFromType(kmriiwa_msgs.LBRStatus._TYPE);
		try
		{
			msg.getHeader().setStamp(time.getCurrentTime());
			msg.setMotionEnabled(robot.isMotionEnabled());
			msg.setAxesMastered(robot.isMastered());
			msg.setAxesGmsReferenced(robot.getSafetyState().areAllAxesGMSReferenced());
			msg.setAxesPositionReferenced(robot.getSafetyState().areAllAxesPositionReferenced());
			msg.setSafetyStateEnabled(robot.getSafetyState().getSafetyStopSignal().compareTo(SafetyStopType.NOSTOP) != 0);
		}
		catch (NullPointerException e)
		{
			Logger.warn("Couldn't retrieve LBR arm status");
			Logger.warn("Empty LBRStatus message is generated");
		}
		return msg;
	}
}
