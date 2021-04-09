package uk.ac.liverpool.lrcfmd.kmriiwa.robot;

import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;

public class KMRCommander {
	
	private KmpOmniMove kmr;
	
	public KMRCommander(KmpOmniMove robot)
	{
		this.kmr = robot;
	}
	
	public void twistJog(geometry_msgs.Twist twistCommand)
	{
		if (twistCommand != null)
		{
			if (kmr.isMotionEnabled() && kmr.isReadyToMove())
			{
				try
				{
					double vel[] = {twistCommand.getLinear().getX(), twistCommand.getLinear().getY(), twistCommand.getAngular().getZ()};
					kmr.jog(vel);
					Thread.sleep(50);
				}
				catch (InterruptedException e)
				{
					e.printStackTrace();
				}
			}
		}
	}

}
