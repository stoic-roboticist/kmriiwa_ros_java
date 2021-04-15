package uk.ac.liverpool.lrcfmd.kmriiwa.robot;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.SplineJP;

import control_msgs.FollowJointTrajectoryActionGoal;

import trajectory_msgs.JointTrajectoryPoint;
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
	
	public void followJointTrajectory(FollowJointTrajectoryActionGoal trajectoryGoal, DestinationReachedListener motionListner)
	{
		if (trajectoryGoal != null)
		{
			int numJoints = trajectoryGoal.getGoal().getTrajectory().getJointNames().size();
			if (numJoints == 7)
			{
				List<JointTrajectoryPoint> trajectoryPoints = trajectoryGoal.getGoal().getTrajectory().getPoints();
				if (trajectoryPoints.size() > 500)
				{
					throw new RuntimeException("Number of trajectory points exceed the SPlineJP limit of 500 ptp points");
				}
				ListIterator<JointTrajectoryPoint> trajectoryPointsIterator = trajectoryPoints.listIterator();
				ArrayList<PTP> ptpList = new ArrayList<PTP>();
				while (trajectoryPointsIterator.hasNext())
				{
					JointTrajectoryPoint trajectoryPoint = trajectoryPointsIterator.next();
					double[] positions = trajectoryPoint.getPositions();
					ptpList.add(ptp(new JointPosition(positions)));
					//double[] velocities = trajectoryPoint.getVelocities();
					//double[] accelerations = trajectoryPoint.getAccelerations();
					//ptpList.add(ptp(new JointPosition(positions)).setJointVelocityRel(velocities).setJointAccelerationRel(accelerations));
				}
				
				PTP[] ptpArray = new PTP[ptpList.size()];
				ptpList.toArray(ptpArray);
				SplineJP trajectory = new SplineJP(ptpArray);
				robot.move(trajectory, motionListner);
			}
		}
	}
}
