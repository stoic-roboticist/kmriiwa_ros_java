package uk.ac.liverpool.lrcfmd.kmriiwa.app;

import java.net.InetAddress;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.UnknownHostException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.ros.address.BindAddress;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.TimeProvider;
import org.ros.time.NtpTimeProvider;
import org.ros.time.WallTimeProvider;

import control_msgs.FollowJointTrajectoryActionGoal;

import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.ActionServerNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.ActionServerNode.Goal;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.PublicationNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.SubscriptionNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.ToolNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.KMRCommander;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.KMRMsgGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRCommander;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRMsgGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.AddressGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.DestinationReachedListener;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.Logger;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.PublisherTask;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.persistenceModel.PersistenceException;


public class ROSKmriiwaController extends RoboticsAPIApplication {
	// KUKA robot controllers
	private LBR robotArm = null;
	private KmpOmniMove robotBase = null;
	
	// Application flags
	private boolean initSuccessful = false;
	private boolean running = false;
	private volatile boolean paused = false;
	
	// ROS nodes for communication
	private SubscriptionNode subscriber = null;
	private PublicationNode publisher = null;
	private ActionServerNode actionServer = null;
	private ScheduledExecutorService taskRunner = null;
	private ScheduledExecutorService ntpExecutorService = null;
	
	// ROS configurations
	private NodeConfiguration subscriberNodeConfiguration = null;
	private NodeConfiguration publisherNodeConfiguration = null;
	private NodeConfiguration actionNodeConfiguration = null;
	private NodeConfiguration toolNodeConfiguration = null;
	private TimeProvider timeProvider = null;
	private AddressGenerator addressGenerator = new AddressGenerator();
	private NodeMainExecutor nodeMainExecutor = null;
	private String robotName;
	private String masterIP;
	private String masterPort;
	private String robotIP;
	private String masterUri;
	
	// Robot interfaces
	private LBRMsgGenerator lbrMsgGenerator = null;
	private LBRCommander lbrCommander = null;
	private KMRMsgGenerator kmrMsgGenerator = null;
	private KMRCommander kmrCommander = null;
	private ToolNode toolNode = null;
	
	@Override
	public void initialize()
	{
		// Initialise the robot controllers	
		robotArm = getContext().getDeviceFromType(LBR.class);
		robotBase = getContext().getDeviceFromType(KmpOmniMove.class);
		
		// Initialise the logger
		Logger.setLogger(getLogger());
		
		// Initialise ROS master and time provider settings
		configureRosMaster();
		configureTimeProvider();
		
		// Initialise KUKA arm and base commanders and message generators
		lbrMsgGenerator = new LBRMsgGenerator(robotArm, robotName, timeProvider);
		lbrCommander = new LBRCommander(robotArm);
		kmrMsgGenerator = new KMRMsgGenerator(robotBase,robotName, timeProvider);
		kmrCommander = new KMRCommander(robotBase);
		
		subscriber = new SubscriptionNode(robotName);
		publisher = new PublicationNode(robotName);
		actionServer = new ActionServerNode(robotName);
		// Initialise tooNode if available
		//toolNode = new FestoGripperNode(robotName, "festoGripper");
		
		// ROS nodes initialisation
		try
		{
			subscriberNodeConfiguration = configureNode(subscriber.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
					addressGenerator.getNewAddress());
			publisherNodeConfiguration = configureNode(publisher.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
					addressGenerator.getNewAddress());
			actionNodeConfiguration = configureNode(actionServer.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
					addressGenerator.getNewAddress());
			
			if (toolNode != null)
			{
				toolNodeConfiguration = configureNode(toolNode.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
						addressGenerator.getNewAddress());
			}
		}
		catch (Exception e)
		{
			Logger.error("Error when initializing ROS nodes");
			Logger.error(e.getMessage());
			return;
		}
		
		try
		{
			nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			
			nodeMainExecutor.execute(publisher, publisherNodeConfiguration);
			nodeMainExecutor.execute(subscriber, subscriberNodeConfiguration);
			nodeMainExecutor.execute(actionServer, actionNodeConfiguration);
			Logger.info("ROS node executor initialized");
			
			if (toolNode != null)
			{
				nodeMainExecutor.execute(toolNode, toolNodeConfiguration);
				Logger.info("Tool node executor initialized");
			}
			
		}
		catch (Exception e)
		{
			Logger.error("Error when starting ROS node executor");
			Logger.error(e.getMessage());
			return;
		}
		// end of ROS nodes initialisation
		initSuccessful = true;
	}

	@Override
	public void run() throws Exception {
		if (!initSuccessful) { throw new RuntimeException("Could not initialize the RoboticApplication successfully."); }
		
		//wait for ROS master
		Logger.warn("waiting for ROS master");
		Logger.warn("Application will terminate in 2 min if no master connects");
		long startTime = System.currentTimeMillis();
		while (!allNodesConnected())
		{
			if (System.currentTimeMillis() - startTime > 120000)
			{
				Logger.error("couldn't connect to master, exiting!!!");
				nodeMainExecutor.shutdown();
				nodeMainExecutor.getScheduledExecutorService().shutdownNow();
				return;
			}
		}
		Logger.info("all nodes connected to the ROS master");
		
		// subscribe to FDI to get laser and odometry data
		kmrMsgGenerator.subscribeToSensors(10000);
		
		running = true;
		
		// start the publisher thread
		try
		{
			PublisherTask publisherTask = new PublisherTask(publisher, lbrMsgGenerator, kmrMsgGenerator);
			taskRunner = Executors.newSingleThreadScheduledExecutor();
			taskRunner.scheduleAtFixedRate(publisherTask, 0, 100, TimeUnit.MILLISECONDS);
			// add toolNode to the task runner to publish gripper state
			if (toolNode != null)
			{
				taskRunner.scheduleAtFixedRate(new Runnable() {
					@Override
					public void run()
					{
						toolNode.publishToolState();
					}
				}, 0, 100, TimeUnit.MILLISECONDS);
			}
		}
		catch (Exception e)
		{
			Logger.error("Error when starting publisher thread");
			Logger.error(e.getMessage());
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
				if (toolNode != null)
				{
					toolNode.executeToolCommand();
				}
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
	
	private void configureTimeProvider()
	{
		boolean useNtp = getApplicationData().getProcessData("ntp").getValue();
		if (useNtp)
		{
			try
			{
				ntpExecutorService = Executors.newScheduledThreadPool(1);
				timeProvider = new NtpTimeProvider(InetAddress.getByName(masterIP), ntpExecutorService);
				((NtpTimeProvider) timeProvider).startPeriodicUpdates(100, TimeUnit.MILLISECONDS);
				Logger.info("NTP server is used as a time provider");
			}
			catch (UnknownHostException e) {
				Logger.error("Could not setup NTP time provider!");
		    }
		}
		else
		{
			timeProvider = new WallTimeProvider();
			Logger.info("WallTime is used as a time provider");
		}
	}
	
	private void configureRosMaster()
	{
		try
		{
			robotName = getApplicationData().getProcessData("robot_name").getValue();
			masterIP = getApplicationData().getProcessData("master_ip").getValue();
			masterPort = getApplicationData().getProcessData("master_port").getValue();
			robotIP = getApplicationData().getProcessData("robot_ip").getValue();
			masterUri = "http://" + masterIP + ":" + masterPort;
		}
		catch (PersistenceException e)
		{
			Logger.error("Application data not set correctly. Please check the package Readme document");
		}
	}
	
	private boolean allNodesConnected()
	{
		// !publisher.isConnectedToMaster() || !subscriber.isConnectedToMaster() || !actionServer.isConnectedToMaster() 
		//|| (!toolNode.isConnectedToMaster() && toolNode!=null)
		boolean allConnected = publisher.isConnectedToMaster() && subscriber.isConnectedToMaster() && actionServer.isConnectedToMaster();
		if (toolNode != null)
		{
			allConnected = allConnected && toolNode.isConnectedToMaster();
		}
		return allConnected;
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
				kmriiwa_msgs.JointPosition jpTarget = subscriber.getJointPositionTarget();
				if (jpTarget != null)
				{
					lbrCommander.moveToJointPosition(jpTarget, new DestinationReachedListener(publisher));
				}
			}
		}
		catch (Exception e)
		{
			Logger.error("Couldn't execute LBR command");
			Logger.error(e.toString());
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
			Logger.error("Couldn't execute KMR command");
			Logger.error(e.toString());
			e.printStackTrace();
		}
	}
	
	@Override
	public void dispose()
	{
		Logger.warn("Shutting down ROS KMRIIWA controller");
		running = false;
		// stop the publisher thread
		shutDownExecutor(ntpExecutorService);
		shutDownExecutor(taskRunner);
		// close fdi connection
		kmrMsgGenerator.close();
		
		// shutdown ROS node executor
		if (nodeMainExecutor != null) 
		{
			Logger.info("Shutting down ROS node executor");
			nodeMainExecutor.shutdown();
			nodeMainExecutor.getScheduledExecutorService().shutdownNow();
		}
		Logger.info("All shutdown cleanly");
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
				Logger.info("Executor service terminated cleanly");
			}
			catch (InterruptedException e)
			{
				Logger.error(e.getMessage());
				executor.shutdownNow();
			}
		}
	}
	
	@Override
	public void onApplicationStateChanged(RoboticsAPIApplicationState state) 
	{
		if (state == RoboticsAPIApplicationState.STOPPING) 
	    {
			Logger.warn("ROS KMRIIWA conttroller Application is stopping");
			running = false;
			paused = true;
	    }
		else if (state == RoboticsAPIApplicationState.MOTIONPAUSING) 
	    {
			Logger.warn("ROS KMRIIWA conttroller Application is pausing");
			paused = true;
	    }
		else if (state == RoboticsAPIApplicationState.RESUMING) 
	    {
			Logger.warn("ROS KMRIIWA conttroller Application is resuming");
			paused = false;
	    }
	}
}
