package uk.ac.liverpool.lrcfmd.kmriiwa.robot;

import tool.GripperFesto;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.PublicationNode;

public class GripperCommander {
	
	private GripperFesto gripper;
	private Boolean isGrpOpen = null;
	
	public GripperCommander(GripperFesto gripperFesto)
	{
		this.gripper = gripperFesto;
		System.out.println("ensure to home the gripper before usage");
	}
	
	public void openGripper (std_msgs.Bool openCmd, PublicationNode pubNode)
	{
		if (openCmd != null)
		{
			if (openCmd.getData())
			{
				gripper.driveToPosition(2);
				isGrpOpen = true;
			}
			else
			{
				gripper.prepareSensitiveMove((Integer)8 );
				
				gripper.moveSensitive(1);
				isGrpOpen = false;
			}
			// need to check if gripper executed
			pubNode.publishGripperDestinationReached();
		}
	}

}
