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

import uk.ac.liverpool.lrcfmd.kmriiwa.utility.Conversions;

public class LBRMsgGenerator {
	
	private LBR robot;
	private String robotName;
	
	// Needed to create ROS messages
	private NodeConfiguration nodeConf = NodeConfiguration.newPrivate();
	private MessageFactory messageFactory = nodeConf.getTopicMessageFactory();
	private TimeProvider time;
	
	private String baseFrameID;
	private static final String BASE_FRAME_SUFFIX = "_some_link";
	
	private String[] joint_names;
	
	
	/**
	 * 
	 * @param lbr
	 * @param name
	 * @param timeProvider
	 */
	public LBRMsgGenerator(LBR lbr, String name, TimeProvider timeProvider)
	{
		this.robot = lbr;
		this.robotName = name;
		this.baseFrameID = robotName + BASE_FRAME_SUFFIX;
		this.time = timeProvider;
		
		joint_names = new String[] { robotName + "_joint_1", robotName + "_joint_2", robotName + "_joint_3", 
				robotName + "_joint_4", robotName + "_joint_5", robotName + "_joint_6",
		        robotName + "_joint_7" };
	}
	
	/**
	 * 
	 * @param frame
	 * @return
	 */
	public iiwa_msgs.CartesianPose getCurrentCartesianPose()
	{
		Frame cartesianFrame = robot.getCurrentCartesianPosition(robot.getFlange());
		Transformation transform = cartesianFrame.transformationFromWorld();
		LBRE1Redundancy redundancy = (LBRE1Redundancy) cartesianFrame.getRedundancyInformationForDevice(robot);
		iiwa_msgs.CartesianPose msg = messageFactory.newFromType(iiwa_msgs.CartesianPose._TYPE);
		
		Conversions.kukaTransformationToRosPose(transform, msg.getPoseStamped().getPose());
		msg.getPoseStamped().getHeader().setFrameId(baseFrameID);
		msg.getPoseStamped().getHeader().setStamp(time.getCurrentTime());
		msg.getRedundancy().setE1(redundancy.getE1());
		msg.getRedundancy().setStatus(redundancy.getStatus());
	    msg.getRedundancy().setTurn(redundancy.getTurn());
	    
	    return msg;
	}
	
	public iiwa_msgs.JointPosition getCurrentJointPosition()
	{
		iiwa_msgs.JointPosition msg = messageFactory.newFromType(iiwa_msgs.JointPosition._TYPE);
		double[] position = robot.getCurrentJointPosition().getInternalArray();
		msg.getHeader().setStamp(time.getCurrentTime());
	    Conversions.vectorToJointQuantity(position, msg.getPosition());
	    return msg;
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
	
	public std_msgs.Time getCurrentTime()
	{
		std_msgs.Time msg = messageFactory.newFromType(std_msgs.Time._TYPE);
		
		msg.setData(time.getCurrentTime());
		
		return msg;
	}
}
