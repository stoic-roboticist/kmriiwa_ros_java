package uk.ac.liverpool.lrcfmd.kmriiwa.app;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.ros.address.BindAddress;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.TimeProvider;
import org.ros.time.WallTimeProvider;

import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.PublicationNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.nodes.SubscriptionNode;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRCommander;
import uk.ac.liverpool.lrcfmd.kmriiwa.robot.LBRMsgGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.AddressGenerator;
import uk.ac.liverpool.lrcfmd.kmriiwa.utility.PublisherTask;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.deviceModel.LBR;

public class ROSKmriiwaController extends RoboticsAPIApplication {
	
	private LBR robotArm = null;
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
	
	@Override
	public void initialize()
	{
		robotArm = getContext().getDeviceFromType(LBR.class);
		
		timeProvider = new WallTimeProvider();
		
		lbrMsgGenerator = new LBRMsgGenerator(robotArm, robotName, timeProvider);
		lbrCommander = new LBRCommander(robotArm);
		
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
			System.out.println(e.toString());
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
			System.out.println(e.toString());
			e.printStackTrace();
			return;
		}
		// end of ROS nodes initialisation
		
		initSuccessful = true;
	}

	@Override
	public void run() throws Exception {
		if (!initSuccessful) { throw new RuntimeException("Could not initialize the RoboticApplication successfully."); }
		
		running = true;
		
		// start the publisher thread
		PublisherTask publisherTask = new PublisherTask(publisher, lbrMsgGenerator);
		taskRunner = Executors.newSingleThreadScheduledExecutor();
		taskRunner.scheduleAtFixedRate(publisherTask, 0, 500, TimeUnit.MILLISECONDS);
		
		while (running)
		{
			executeCommands();
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
	
	private void executeCommands()
	{
		try
		{
			iiwa_msgs.JointPosition jpTarget = subscriber.getJointPositionTarget();
			if (jpTarget != null)
			{
				lbrCommander.moveToJointPosition(jpTarget);
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
		
		// shutdown ROS node executor
		if (nodeMainExecutor != null) 
		{
			System.out.println("Shutting down ROS executor");
			nodeMainExecutor.shutdown();
			try
			{
				nodeMainExecutor.getScheduledExecutorService().awaitTermination(5, TimeUnit.SECONDS);
			}
			catch (InterruptedException e)
			{
				System.out.println(e.getMessage());
				nodeMainExecutor.getScheduledExecutorService().shutdownNow();
			}
		}
	}
	
	private void shutDownExecutor(ScheduledExecutorService executor)
	{
		executor.shutdown();
		try
		{
			executor.awaitTermination(5, TimeUnit.SECONDS);
		}
		catch (InterruptedException e)
		{
			System.out.println(e.getMessage());
			executor.shutdownNow();
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
	    super.onApplicationStateChanged(state);
	}
}
