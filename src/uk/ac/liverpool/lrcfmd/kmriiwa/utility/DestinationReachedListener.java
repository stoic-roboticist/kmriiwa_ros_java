package uk.ac.liverpool.lrcfmd.kmriiwa.utility;

import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.ActionServerNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.PublicationNode;

import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.executionModel.IExecutionContainer;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainerListener;

public class DestinationReachedListener implements IMotionContainerListener {
	
	private PublicationNode publisher = null;
	private ActionServerNode actionServer = null;
	
	public DestinationReachedListener(PublicationNode publisher, ActionServerNode actionServer)
	{
		this.publisher = publisher;
		this.actionServer = actionServer;
	}
	
	public DestinationReachedListener(PublicationNode publisher)
	{
		this.publisher = publisher;
	}

	@Override
	public void onStateChanged(IExecutionContainer arg0, ExecutionState arg1) 
	{
		// not used
	}

	@Override
	public void containerFinished(IMotionContainer container) 
	{
	    if (publisher != null) 
	    {
	    	publisher.publishArmDestinationReached();
	    }
	    if (actionServer != null  && actionServer.hasCurrentGoal() && actionServer.isActive()) 
	    {
	    	if (!container.hasError())
	    	{
	    		actionServer.markCurrentGoalReached();
	    	}
	    	else
	    	{
	    		actionServer.markCurrentGoalFailed(container.getErrorMessage());
	    	}
	    }
	}

	@Override
	public void motionFinished(IMotion motion) 
	{
		// Not used
		// Test to see if feedback can be sent		
	}

	@Override
	public void motionStarted(IMotion motion) 
	{
		// Not used
	}

}
