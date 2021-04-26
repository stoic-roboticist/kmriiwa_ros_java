package uk.ac.liverpool.lrcfmd.kmriiwa.nodes;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.ros.internal.message.Message;
import control_msgs.FollowJointTrajectoryActionGoal;
import control_msgs.FollowJointTrajectoryActionFeedback;
import control_msgs.FollowJointTrajectoryActionResult;
import control_msgs.FollowJointTrajectoryResult;
import trajectory_msgs.JointTrajectoryPoint;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.Logger;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;

import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;

import com.github.rosjava_actionlib.ActionServer;
import com.github.rosjava_actionlib.ActionServerListener;

public class ActionServerNode extends AbstractNodeMain {
	
	public class Goal<T_ACTION_GOAL extends Message> 
	{
	    public T_ACTION_GOAL goal = null;
	    public String goalId = null;

	    public Goal(T_ACTION_GOAL goal, String goalId) 
	    {
	      this.goal = goal;
	      this.goalId = goalId;
	    }
	}
	
	public abstract class simpleActionServerListener<T_ACTION_GOAL extends Message> implements ActionServerListener<T_ACTION_GOAL> 
	{
	    private ActionServerNode server = null;
	    public simpleActionServerListener(ActionServerNode server) 
	    {
	      this.server = server;
	    }

	    @Override
	    public boolean acceptGoal(T_ACTION_GOAL goal) 
	    {
	      return true;
	    }

	    @Override
	    public void cancelReceived(GoalID goalId) 
	    {
	      server.markCurrentGoalFailed("Goal execution canceled by client.");
	    }

	    @Override
	    public void goalReceived(T_ACTION_GOAL goal) 
	    {
	      synchronized (server) {
	        server.goalQueue.add(new Goal<T_ACTION_GOAL>(goal, this.getGoalId(goal)));
	      }
	    }

	    public abstract String getGoalId(T_ACTION_GOAL goal);
	}
	
	private ConnectedNode node = null;
	private String robotName = "kmriiwa";
	private boolean connectedToMaster = false;
	
	Queue<Goal<?>> goalQueue;
	Goal<?> currentGoal;
	
	private ActionServer<FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult> followTrajectoryServer = null;
	
	public ActionServerNode(String robotName) 
	{
		this.robotName = robotName;
		goalQueue = new LinkedBlockingQueue<ActionServerNode.Goal<?>>();
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(robotName + "/follow_joint_trajectory_action_server");
	}
	
	@Override
	public synchronized void onStart(ConnectedNode connectedNode) 
	{
		node = connectedNode;
		goalQueue.clear();
		
		followTrajectoryServer = new ActionServer<FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult>(node, robotName
	            + "/arm/manipulator_controller/follow_joint_trajectory", FollowJointTrajectoryActionGoal._TYPE, FollowJointTrajectoryActionFeedback._TYPE, FollowJointTrajectoryActionResult._TYPE);
	    
		followTrajectoryServer.attachListener(new simpleActionServerListener<FollowJointTrajectoryActionGoal>(this) {
	        @Override
	        public String getGoalId(FollowJointTrajectoryActionGoal goal) 
	        {
	          return goal.getGoalId().getId();
	        }
	      });
	    connectedToMaster = true;
	}
	
	 @Override
	  public void onShutdown(Node node) 
	 {
		 followTrajectoryServer.finish();
		 goalQueue.clear();
		 goalQueue = null;
	  }
	
	private synchronized void markCurrentGoal(boolean succeeded, String error_msg) 
	{
		if (hasCurrentGoal()) 
	    {
	          FollowJointTrajectoryActionResult result = followTrajectoryServer.newResultMessage();
	          result.getResult().setErrorString(error_msg);
	          result.getStatus().getGoalId().setId(currentGoal.goalId);
	          if (succeeded) 
	          {
	        	result.getResult().setErrorCode(FollowJointTrajectoryResult.SUCCESSFUL);
	            result.getStatus().setStatus(GoalStatus.SUCCEEDED);
	            followTrajectoryServer.setSucceed(currentGoal.goalId);
	          }
	          else 
	          {
	        	result.getResult().setErrorCode(FollowJointTrajectoryResult.INVALID_GOAL);
	            result.getStatus().setStatus(GoalStatus.ABORTED);
	            followTrajectoryServer.setAborted(currentGoal.goalId);
	          }
	          followTrajectoryServer.sendResult(result);
	          followTrajectoryServer.setGoalStatus(result.getStatus(), currentGoal.goalId);
        }

	      currentGoal = null;
	      
	}
	
	public void markCurrentGoalReached() 
	{
		if (hasCurrentGoal()) 
		{
	      Logger.info("Goal reached: " + currentGoal.goalId);
	    }
	    markCurrentGoal(true, "");
	}

	public void markCurrentGoalFailed(String error_msg) 
	{
		if (hasCurrentGoal()) 
		{
			Logger.warn("Goal failed: " + currentGoal.goalId);
			Logger.warn("Reason: " + error_msg);
	    }
	    markCurrentGoal(false, error_msg);
	}
	
	public Boolean isActive() 
	{
	    return followTrajectoryServer != null;
	}
	
	public synchronized boolean newGoalAvailable() 
	{
	    if (goalQueue != null && isActive()) 
	    {
	      return !goalQueue.isEmpty();
	    }
	    else 
	    {
	      return false;
	    }
	  }

	public synchronized boolean hasCurrentGoal() 
	{
		return currentGoal != null;
	}

	public synchronized Goal<?> getCurrentGoal() 
	{
	    return currentGoal;
	}

	public synchronized Goal<?> getNextGoal() 
	{
	    return goalQueue.peek();
	}

	public synchronized Goal<?> acceptNewGoal() 
	{
	    currentGoal = goalQueue.poll();
	    return currentGoal;
	}

	public synchronized void publishCurrentState() 
	{
		if (hasCurrentGoal()) 
	    {
			followTrajectoryServer.sendStatusTick();
	    }
    }
	
	public synchronized void publishFeedback(GoalID goal_id, String frame_id, JointTrajectoryPoint desired, JointTrajectoryPoint actual, JointTrajectoryPoint error) 
	{
		FollowJointTrajectoryActionFeedback feedback = followTrajectoryServer.newFeedbackMessage();
		// header
		feedback.getHeader().setFrameId(frame_id);
		feedback.getHeader().setStamp(node.getCurrentTime());
		
		// feedback
		feedback.getFeedback().setDesired(desired);
		feedback.getFeedback().setError(error);
		feedback.getFeedback().setActual(actual);
		feedback.getFeedback().setHeader(feedback.getHeader());
		
		// status
		feedback.getStatus().setGoalId(goal_id);
		
		followTrajectoryServer.sendFeedback(feedback);
    }
	
	public boolean isConnectedToMaster()
	{
		return connectedToMaster;
	}
}
