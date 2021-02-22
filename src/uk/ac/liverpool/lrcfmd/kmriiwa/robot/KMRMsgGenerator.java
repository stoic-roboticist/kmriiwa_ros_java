package uk.ac.liverpool.lrcfmd.kmriiwa.robot;

import com.kuka.nav.fdi.FDIConnection;
import com.kuka.nav.fdi.data.Odometry;
import com.kuka.nav.provider.LaserScan;
import java.net.InetSocketAddress;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.time.TimeProvider;

public class KMRMsgGenerator {
	
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
	
	public KMRMsgGenerator(TimeProvider timeProvider)
	{
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
	
	public geometry_msgs.Pose getBasePose()
	{
		geometry_msgs.Pose msg = messageFactory.newFromType(geometry_msgs.Pose._TYPE);
		
		if (fdi.getSubscription().isOdometrySubscribed())
		{
			Odometry odometry = fdi.getNewOdometry();
			msg.getPosition().setX(odometry.getPose().getX());
			msg.getPosition().setY(odometry.getPose().getY());
			msg.getPosition().setZ(0);
			
			double quat[] = euler_to_quaternion(0,0,odometry.getPose().getTheta());
			msg.getOrientation().setX(quat[0]);
			msg.getOrientation().setY(quat[1]);
			msg.getOrientation().setZ(quat[2]);
			msg.getOrientation().setW(quat[3]);
			
		}
		else
		{
			System.out.println("FDI not connected to odometry");
		}
		
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
