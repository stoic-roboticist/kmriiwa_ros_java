package uk.ac.liverpool.lrcfmd.kmriiwa.utility;

import com.kuka.task.ITaskLogger;

public class Logger {
	
	private static ITaskLogger sunriseLogger = null;
	
	public static void setLogger(ITaskLogger logger) 
	{
	    sunriseLogger = logger;
    }
	
	public static void info(String message) 
	{
		sunriseLogger.info(message);
	}
	
	public static void warn(String message) 
	{
		sunriseLogger.warn(message);
	}
	
	public static void error(String message) 
	{
		sunriseLogger.error(message);
	}

}
