package uk.ac.liverpool.lrcfmd.kmriiwa.app;

import java.net.InetAddress;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.UnknownHostException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import org.ros.address.BindAddress;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.TimeProvider;
import org.ros.time.NtpTimeProvider;
import org.ros.time.WallTimeProvider;

import tool.GripperFesto;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.ActionServerNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.ActionServerNode.Goal;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.PublicationNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.SubscriptionNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.GripperCommander;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.KMRCommander;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.KMRMsgGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRCommander;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRMsgGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.AddressGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.DestinationReachedListener;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.PublisherTask;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;

import control_msgs.FollowJointTrajectoryActionGoal;

public class ROSKmriiwaController extends RoboticsAPIApplication {
	private LBR robotArm = null;
	@Inject
	private GripperFesto gripper;
	private KmpOmniMove robotBase = null;
	private String robotName = "kmriiwa";
	
	private boolean initSuccessful = false;
	private boolean running = false;
	private volatile boolean paused = false;
	
	// ROS nodes for communication
	private SubscriptionNode subscriber = null;
	private PublicationNode publisher = null;
	private ActionServerNode actionServer = null;
	private ScheduledExecutorService taskRunner = null;
	private ScheduledExecutorService ntpExecutorService = null;
	
	// ROS configuration setting
	private NodeConfiguration subscriberNodeConfiguration = null;
	private NodeConfiguration publisherNodeConfiguration = null;
	private NodeConfiguration actionNodeConfiguration = null;
	private TimeProvider timeProvider = null;
	private AddressGenerator addressGenerator = new AddressGenerator();
	private NodeMainExecutor nodeMainExecutor = null;
	
	// Define robot and master IP
	private final String masterIP = "172.31.1.77";
	private final String masterPort = "11311";
	private final String robotIP = "172.31.1.10";
	private final String masterUri = "http://" + masterIP + ":" + masterPort;
	
	// Robot interfaces
	private LBRMsgGenerator lbrMsgGenerator = null;
	private LBRCommander lbrCommander = null;
	private GripperCommander festoCommander = null;
	private KMRMsgGenerator kmrMsgGenerator = null;
	private KMRCommander kmrCommander = null;
	
	@Override
	public void initialize()
	{
		robotArm = getContext().getDeviceFromType(LBR.class);
		robotBase = getContext().getDeviceFromType(KmpOmniMove.class);
		gripper.attachTo(robotArm.getFlange());
		
		//timeProvider = new WallTimeProvider();
		try
		{
			ntpExecutorService = Executors.newScheduledThreadPool(1);
			timeProvider = new NtpTimeProvider(InetAddress.getByName(masterIP), ntpExecutorService);
			((NtpTimeProvider) timeProvider).startPeriodicUpdates(100, TimeUnit.MILLISECONDS);
		}
		catch (UnknownHostException e) {
	        System.out.println("Could not setup NTP time provider!");
	    }
		
		lbrMsgGenerator = new LBRMsgGenerator(robotArm, robotName, timeProvider);
		lbrCommander = new LBRCommander(robotArm);
		festoCommander = new GripperCommander(gripper);
		
		kmrMsgGenerator = new KMRMsgGenerator(robotBase, timeProvider);
		kmrCommander = new KMRCommander(robotBase);
		
		
		subscriber = new SubscriptionNode(robotName);
		publisher = new PublicationNode(robotName);
		actionServer = new ActionServerNode(robotName);
		
		// ROS nodes initialisation
		try
		{
			subscriberNodeConfiguration = configureNode(subscriber.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
					addressGenerator.getNewAddress());
			publisherNodeConfiguration = configureNode(publisher.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
					addressGenerator.getNewAddress());
			actionNodeConfiguration = configureNode(actionServer.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
					addressGenerator.getNewAddress());
		}
		catch (Exception e)
		{
			System.out.println("Error when initializing ROS nodes");
			System.out.println(e.getMessage());
			e.printStackTrace();
			return;
		}
		
		try
		{
			nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			
			nodeMainExecutor.execute(publisher, publisherNodeConfiguration);
			nodeMainExecutor.execute(subscriber, subscriberNodeConfiguration);
			nodeMainExecutor.execute(actionServer, actionNodeConfiguration);
			System.out.println("ROS node executor initialized");
		}
		catch (Exception e)
		{
			System.out.println("Error when starting ROS node executor");
			System.out.println(e.getMessage());
			e.printStackTrace();
			return;
		}
		// end of ROS nodes initialisation
		
		initSuccessful = true;
	}

	@Override
	public void run() throws Exception {
		if (!initSuccessful) { throw new RuntimeException("Could not initialize the RoboticApplication successfully."); }
		
		//wait for ROS master
		System.out.println("waiting for the master");
		long startTime = System.currentTimeMillis();
		while (!publisher.isConnectedToMaster() || !subscriber.isConnectedToMaster() || !actionServer.isConnectedToMaster())
		{
			if (System.currentTimeMillis() - startTime > 5000)
			{
				System.out.println("couldn't connect to master, exiting!!!");
				nodeMainExecutor.shutdown();
				nodeMainExecutor.getScheduledExecutorService().shutdownNow();
				return;
			}
		}
		System.out.println("all connected to the ROS master");
		
		// subscribe to FDI to get laser and odometry data
		kmrMsgGenerator.subscribeToSensors(10000);
		
		running = true;
		
		// start the publisher thread
		try
		{
			PublisherTask publisherTask = new PublisherTask(publisher, lbrMsgGenerator, kmrMsgGenerator);
			taskRunner = Executors.newSingleThreadScheduledExecutor();
			taskRunner.scheduleAtFixedRate(publisherTask, 0, 100, TimeUnit.MILLISECONDS);
		}
		catch (Exception e)
		{
			System.out.println("Error when starting publisher thread");
			System.out.println(e.getMessage());
			e.printStackTrace();
			taskRunner.shutdownNow();
			running = false;
		}
		
		while (running)
		{
			if (!paused)
			{
				executeKMRCmd();
				executeLBRCmd();
				executeGripperCmd();
			}
			else
			{
				Thread.sleep(50);
			}
		}


	}
	
	private NodeConfiguration configureNode(String nodeName, int tcpPort, int xmlPort) throws URISyntaxException
	{
		NodeConfiguration nodeConfig = NodeConfiguration.newPublic(robotIP);
		nodeConfig.setTimeProvider(timeProvider);
		nodeConfig.setNodeName(nodeName);
		nodeConfig.setMasterUri(new URI(masterUri));
		nodeConfig.setTcpRosBindAddress(BindAddress.newPublic(tcpPort));
		nodeConfig.setXmlRpcBindAddress(BindAddress.newPublic(xmlPort));
		return nodeConfig;
	}
	
	private void executeLBRCmd()
	{
		try
		{
			if (actionServer.newGoalAvailable())
			{
				actionServer.acceptNewGoal();
				if(actionServer.hasCurrentGoal())
				{
					Goal<?> goal = actionServer.getCurrentGoal();
					// pass the goal to LBRCommander along with the actionServer
					FollowJointTrajectoryActionGoal jointTrajectoryGoal = ((FollowJointTrajectoryActionGoal) goal.goal);
					lbrCommander.followJointTrajectory(jointTrajectoryGoal, new DestinationReachedListener(publisher, actionServer));
				}
			}
			else
			{
				iiwa_msgs.JointPosition jpTarget = subscriber.getJointPositionTarget();
				if (jpTarget != null)
				{
					lbrCommander.moveToJointPosition(jpTarget, new DestinationReachedListener(publisher));
				}
			}
		}
		catch (Exception e)
		{
			System.out.println(e.toString());
			e.printStackTrace();
		}
	}
	
	private void executeGripperCmd()
	{
		try
		{
			std_msgs.Bool openGrp = subscriber.getOpenGripperCmd();
			if (openGrp != null)
			{
				System.out.println("In app gripper msg to exec:" + openGrp.getData());
				festoCommander.openGripper(openGrp,publisher);
			}
		}
		catch (Exception e)
		{
			System.out.println(e.toString());
			e.printStackTrace();
		}
	}
	
	private void executeKMRCmd()
	{
		try
		{
			geometry_msgs.Twist baseTwistTarget = subscriber.getBaseTwistTarget();
			if (baseTwistTarget != null)
			{
				kmrCommander.twistJog(baseTwistTarget);
			}
		}
		catch (Exception e)
		{
			System.out.println(e.toString());
			e.printStackTrace();
		}
	}
	
	@Override
	public void dispose()
	{
		running = false;
		// stop the publisher thread
		shutDownExecutor(ntpExecutorService);
		shutDownExecutor(taskRunner);
		// close fdi connection
		kmrMsgGenerator.close();
		
		// shutdown ROS node executor
		if (nodeMainExecutor != null) 
		{
			System.out.println("Shutting down ROS executor");
			nodeMainExecutor.shutdown();
			nodeMainExecutor.getScheduledExecutorService().shutdownNow();
		}
		
		super.dispose();
	}
	
	private void shutDownExecutor(ScheduledExecutorService executor)
	{
		if (executor != null)
		{
			executor.shutdown();
			try
			{
				executor.awaitTermination(5, TimeUnit.SECONDS);
				System.out.println("Executor service terminated cleanly");
			}
			catch (InterruptedException e)
			{
				System.out.println(e.getMessage());
				executor.shutdownNow();
			}
		}
	}
	
	@Override
	public void onApplicationStateChanged(RoboticsAPIApplicationState state) 
	{
		if (state == RoboticsAPIApplicationState.STOPPING) 
	    {
			running = false;
			paused = true;
	    }
		else if (state == RoboticsAPIApplicationState.MOTIONPAUSING) 
	    {
			paused = true;
	    }
		else if (state == RoboticsAPIApplicationState.RESUMING) 
	    {
			paused = false;
	    }
	}
}
