package uk.ac.liverpool.lrcfmd.kmriiwa.utility;

import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.PublicationNode;

import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.executionModel.IExecutionContainer;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainerListener;

public class DestinationReachedListener implements IMotionContainerListener {
	
	private PublicationNode publisher;
	
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
		System.out.println("Motion finished");
	    if (publisher != null) 
	    {
	      publisher.publishArmDestinationReached();
	    }
	}

	@Override
	public void motionFinished(IMotion motion) 
	{
		// Not used
	}

	@Override
	public void motionStarted(IMotion motion) 
	{
		// Not used
	}

}
