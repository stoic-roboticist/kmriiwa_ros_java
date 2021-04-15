package uk.ac.liverpool.lrcfmd.kmriiwa.robot;

import com.kuka.nav.fdi.FDIConnection;
import com.kuka.nav.fdi.data.Odometry;
import com.kuka.nav.provider.LaserScan;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.SafetyStopType;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;

import java.net.InetSocketAddress;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.time.TimeProvider;

public class KMRMsgGenerator {
	
	private KmpOmniMove robot = null;
	 // Data retrieval socket via FDI 
    private FDIConnection fdi;
	private final String FDI_IP = "172.31.1.102";
	private final int FDI_PORT = 34001;
	
	// Laser ports on controller
	public enum LaserScanner
	{
		LASER_B1(1801, "B1"),
		LASER_B4(1802, "B4");
		
		public final int port;
		public final String id;
		
		private LaserScanner(int port, String id)
		{
			this.port = port;
			this.id = id;
		}
	}
	
	//private final static int LASER_B1 = 1801;
	//private final static int LASER_B4 = 1802;
	
	// Laser scanner constants
	private final float ANGLE_INCREMENT = (float) ((0.5 * Math.PI)/180);
	private final float ANGLE_MIN = (float) ((-135*Math.PI)/180);
	private final float ANGLE_MAX = (float) ((135*Math.PI)/180);
	private final float RANGE_MIN = (float) (0.12);
	private final float RANGE_MAX = (float) (15);
	
	// ROS
	private NodeConfiguration nodeConf = NodeConfiguration.newPrivate();
	private MessageFactory messageFactory = nodeConf.getTopicMessageFactory();
	private TimeProvider time;
	
	public KMRMsgGenerator(KmpOmniMove robot,TimeProvider timeProvider)
	{
		this.robot = robot;
		this.time = timeProvider;
		InetSocketAddress fdi_address = new InetSocketAddress(FDI_IP,FDI_PORT);
		fdi = new FDIConnection(fdi_address);
	}
	
	public void subscribeToSensors(long timeout)
	{	

		fdi.connect();
		if (fdi.isConnected())
		{
			long startTime = System.currentTimeMillis();
			while(!this.fdi.getSubscription().isLaserSubscribed(LaserScanner.LASER_B1.port) && (System.currentTimeMillis() - startTime <= timeout))
			{
				this.fdi.getNewLaserScan(LaserScanner.LASER_B1.port);
			}
			while(!this.fdi.getSubscription().isLaserSubscribed(LaserScanner.LASER_B4.port) && (System.currentTimeMillis() - startTime <= timeout))
			{
				this.fdi.getNewLaserScan(LaserScanner.LASER_B4.port);
			}
			while(!this.fdi.getSubscription().isOdometrySubscribed() && (System.currentTimeMillis() - startTime <= timeout))
			{
				this.fdi.getNewOdometry();
			}
		}
	}
	
	public sensor_msgs.LaserScan getLaserScan(LaserScanner laserScanner)
	{
			sensor_msgs.LaserScan msg = messageFactory.newFromType(sensor_msgs.LaserScan._TYPE);
			
			addLaserScanConstants(msg);
			msg.getHeader().setStamp(time.getCurrentTime());
			msg.getHeader().setFrameId("laser_" + laserScanner.id + "_link");
			if (fdi.getSubscription().isLaserSubscribed(laserScanner.port))
			{
				LaserScan laserScan = fdi.getNewLaserScan(laserScanner.port);
				float ranges[] = laserScan.getScannedRanges();
				msg.setRanges(ranges);
			}
			else
			{
				System.out.println("FDI not connected to laser scanner" + laserScanner.id);
			}
			
			return msg;
	}
	
	public nav_msgs.Odometry getBaseOdometry()
	{
		nav_msgs.Odometry msg = messageFactory.newFromType(nav_msgs.Odometry._TYPE);
		
		if (fdi.getSubscription().isOdometrySubscribed())
		{
			Odometry odometry = fdi.getNewOdometry();
			// Pose with covariance msg
			geometry_msgs.PoseWithCovariance poseWithCov = messageFactory.newFromType(geometry_msgs.PoseWithCovariance._TYPE);
			poseWithCov.getPose().getPosition().setX(odometry.getPose().getX());
			poseWithCov.getPose().getPosition().setY(odometry.getPose().getY());
			poseWithCov.getPose().getPosition().setZ(0.0);
			
			double quatPose[] = euler_to_quaternion(0,0,odometry.getPose().getTheta());
			poseWithCov.getPose().getOrientation().setX(quatPose[0]);
			poseWithCov.getPose().getOrientation().setY(quatPose[1]);
			poseWithCov.getPose().getOrientation().setZ(quatPose[2]);
			poseWithCov.getPose().getOrientation().setW(quatPose[3]);
			
			// Twist with covariance msg
			geometry_msgs.TwistWithCovariance twistWithCov = messageFactory.newFromType(geometry_msgs.TwistWithCovariance._TYPE);
			twistWithCov.getTwist().getLinear().setX(odometry.getVelocity().getX());
			twistWithCov.getTwist().getLinear().setY(odometry.getVelocity().getY());
			twistWithCov.getTwist().getLinear().setZ(0.0);
			
			twistWithCov.getTwist().getAngular().setX(0.0);
			twistWithCov.getTwist().getAngular().setY(0.0);
			twistWithCov.getTwist().getAngular().setZ(odometry.getVelocity().getTheta());
			
			// Odometry msg
			msg.getHeader().setStamp(time.getCurrentTime());
			msg.getHeader().setFrameId("odom");
			msg.setPose(poseWithCov);
			msg.setTwist(twistWithCov);
			msg.setChildFrameId("base_footprint");
		}
		else
		{
			System.out.println("FDI not connected to odometry");
		}
		
		return msg;
	}
	
	public kmriiwa_msgs.KMRStatus getKMRStatus()
	{
		kmriiwa_msgs.KMRStatus msg = messageFactory.newFromType(kmriiwa_msgs.KMRStatus._TYPE);
		
		msg.getHeader().setStamp(time.getCurrentTime());
		msg.setChargeStatePercentage((int) robot.getMobilePlatformBatteryState().getStateOfCharge());
		msg.setWarningFieldClear(robot.getMobilePlatformSafetyState().isWarningFieldBreached());
		msg.setSafetyFieldClear(robot.getMobilePlatformSafetyState().isSafetyFieldBreached());
		msg.setMotionEnabled(robot.isMotionEnabled());
		msg.setSafetyStateEnabled(robot.getSafetyState().getSafetyStopSignal().compareTo(SafetyStopType.NOSTOP) != 0);
		
		return msg;
	}
	
	private void addLaserScanConstants(sensor_msgs.LaserScan msg)
	{
		msg.setAngleIncrement(ANGLE_INCREMENT);
		msg.setAngleMin(ANGLE_MIN);
		msg.setAngleMax(ANGLE_MAX);
		msg.setRangeMin(RANGE_MIN);
		msg.setRangeMax(RANGE_MAX);
	}
	
	private double[] euler_to_quaternion(double roll, double pitch, double yaw)
	{			
		double qx = Math.sin(roll/2) * Math.cos(pitch/2) * Math.cos(yaw/2) - Math.cos(roll/2) * Math.sin(pitch/2) * Math.sin(yaw/2);
        double qy = Math.cos(roll/2) * Math.sin(pitch/2) * Math.cos(yaw/2) + Math.sin(roll/2) * Math.cos(pitch/2) * Math.sin(yaw/2);
        double qz = Math.cos(roll/2) * Math.cos(pitch/2) * Math.sin(yaw/2) - Math.sin(roll/2) * Math.sin(pitch/2) * Math.cos(yaw/2);
        double qw = Math.cos(roll/2) * Math.cos(pitch/2) * Math.cos(yaw/2) + Math.sin(roll/2) * Math.sin(pitch/2) * Math.sin(yaw/2);
        
		return new double[]{qx,qy,qz,qw};  
	}
	
	public void close()
	{
		try
		{
			if (fdi.isConnected())
			{
				fdi.disconnect();
			}
		}
		catch (Exception e)
		{
			System.out.println("couldn't disconnect FDI socket");
		}
	}

}
