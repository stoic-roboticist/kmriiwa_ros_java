package uk.ac.liverpool.lrcfmd.kmriiwa.robot;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PTP;

import uk.ac.liverpool.lrcfmd.kmriiwa.utility.Conversions;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.DestinationReachedListener;

public class LBRCommander {
	
	private LBR robot;
	double defaultExecVelocity = 0.2;
	
	public LBRCommander(LBR robot)
	{
		this.robot = robot;
	}
	
	public void moveToJointPosition(iiwa_msgs.JointPosition commandPosition, DestinationReachedListener motionListner)
	{
		if (commandPosition != null)
		{
			JointPosition jp = new JointPosition(robot.getJointCount());
			Conversions.rosJointQuantityToKuka(commandPosition.getPosition(), jp);
			PTP ptpMotion = ptp(jp);
			robot.move(ptpMotion.setJointVelocityRel(defaultExecVelocity),motionListner);
		}
	}
}
