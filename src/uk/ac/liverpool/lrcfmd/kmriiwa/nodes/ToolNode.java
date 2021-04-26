package uk.ac.liverpool.lrcfmd.kmriiwa.nodes;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

public abstract class ToolNode extends AbstractNodeMain{
	
	protected String robotName = "kmriiwa";
	protected String toolName;
	protected ConnectedNode node = null;
	protected boolean connectedToMaster = false;
	
	public ToolNode(String robotName, String toolName)
	{
		this.robotName = robotName;
		this.toolName = toolName;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(robotName + "/" + toolName);
	}
	
	public abstract void executeToolCommand();
	public abstract void publishToolState();
	
	public boolean isConnectedToMaster()
	{
		return connectedToMaster;
	}
}
