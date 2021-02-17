package uk.ac.liverpool.lrcfmd.kmriiwa.app;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import org.ros.address.BindAddress;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.TimeProvider;
import org.ros.time.WallTimeProvider;

import tool.GripperFesto;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.PublicationNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.SubscriptionNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.GripperCommander;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.KMRMsgGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRCommander;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRMsgGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.AddressGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.DestinationReachedListener;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.PublisherTask;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.deviceModel.LBR;

public class ROSKmriiwaController extends RoboticsAPIApplication {
	
	private LBR robotArm = null;
	@Inject
	private GripperFesto gripper;
	private String robotName = "kmriiwa";
	
	private boolean initSuccessful = false;
	private boolean running = false;
	
	// ROS nodes for communication
	private SubscriptionNode subscriber = null;
	private PublicationNode publisher = null;
	private ScheduledExecutorService taskRunner = null;
	
	// ROS configuration setting
	private NodeConfiguration subscriberNodeConfiguration = null;
	private NodeConfiguration publisherNodeConfiguration = null;
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
	
	@Override
	public void initialize()
	{
		robotArm = getContext().getDeviceFromType(LBR.class);
		gripper.attachTo(robotArm.getFlange());
		
		timeProvider = new WallTimeProvider();
		
		lbrMsgGenerator = new LBRMsgGenerator(robotArm, robotName, timeProvider);
		lbrCommander = new LBRCommander(robotArm);
		festoCommander = new GripperCommander(gripper);
		
		kmrMsgGenerator = new KMRMsgGenerator(timeProvider);
		kmrMsgGenerator.subscribeToSensors(10000);
		
		subscriber = new SubscriptionNode(robotName);
		publisher = new PublicationNode(robotName);
		
		// ROS nodes initialisation
		try
		{
			subscriberNodeConfiguration = configureNode(subscriber.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
					addressGenerator.getNewAddress());
			publisherNodeConfiguration = configureNode(publisher.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
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
		while (!publisher.isConnectedToMaster() || !subscriber.isConnectedToMaster())
		{
			if (System.currentTimeMillis() - startTime > 5000)
			{
				System.out.println("couldn't connect to master, exiting!!!");
				return;
			}
		}
		System.out.println("all connected to the ROS master");
		
		running = true;
		
		// start the publisher thread
		try
		{
			PublisherTask publisherTask = new PublisherTask(publisher, lbrMsgGenerator, kmrMsgGenerator);
			taskRunner = Executors.newSingleThreadScheduledExecutor();
			taskRunner.scheduleAtFixedRate(publisherTask, 2000, 500, TimeUnit.MILLISECONDS);
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
			executeLBRCmd();
			executeGripperCmd();
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
			iiwa_msgs.JointPosition jpTarget = subscriber.getJointPositionTarget();
			if (jpTarget != null)
			{
				lbrCommander.moveToJointPosition(jpTarget, new DestinationReachedListener(publisher));
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
	
	@Override
	public void dispose()
	{
		running = false;
		// stop the publisher thread
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
	}
	
	private void shutDownExecutor(ScheduledExecutorService executor)
	{
		if (executor != null)
		{
			executor.shutdown();
			try
			{
				executor.awaitTermination(5, TimeUnit.SECONDS);
				System.out.println("publisher thread terminated cleanly");
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
			System.out.println("Controller is stopping");
	    }
		else if (state == RoboticsAPIApplicationState.MOTIONPAUSING) 
	    {
			//running = false;
			System.out.println("Motion is paused");
	    }
	    super.onApplicationStateChanged(state);
	}
}
