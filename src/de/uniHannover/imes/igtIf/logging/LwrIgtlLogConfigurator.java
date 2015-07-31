package de.uniHannover.imes.igtIf.logging;

import java.io.File;
import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;
import java.util.logging.SocketHandler;
import java.util.logging.XMLFormatter;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;

/**
 * This class arranges the logging capabilities for the LWRopenIGTL project. It
 * is based on the java.util.logging classes. It sets up a normal-logger (all
 * output will be directed to the smartPad) and optionally a debug-logger(all
 * output will be directed to a file, Logs all levels). To use the logger you
 * will need to call the setup function. Afterwards you can access globally the
 * logger for this project by calling
 * {@code Logger.getLogger(LwrIgtlLogConfigurator.LOGGERS_NAME)}. Needs to be
 * disposed after usage.
 */
public final class LwrIgtlLogConfigurator {

    /**
     * Enum which indicates how log records are processed.
     */
    public enum LogForwardType {
	/** Log records are send to a file. */
	File, /** Log records are send to a network host. */
	Network;
    }

    /** File object where debug logging output is saved in. */
    public static final File DEBUG_LOGFILE = new File(
	    System.getProperty("user.dir") + File.separator + "logs"
		    + File.separator
		    + StateMachineApplication.class.getSimpleName()
		    + "_debugLog.log");

    /** Name of the logger. */
    public static final String LOGGERS_NAME = StateMachineApplication.class
	    .getSimpleName();

    /** IP of the machine, which runs a Logging-Viewer (for example otros). */
    public static final String DEBUG_VIEWER_IP = "172.31.1.1";
    /** Port of the Logging-Viewer (for example otros). */
    public static final int DEBUG_VIEWER_PORT = 49003;
    /** Singleton instance. */
    private static LwrIgtlLogConfigurator instance;
    /** the file handler for logging output. */
    private FileHandler debugFileHandler;
    /** Debug socket-handler. */
    private SocketHandler debugSocketHandler;
    /** the handler for smartPad console output. */
    private SmartPadLogHandler normalSmartPadHandler;
    /** the logging object. */
    private Logger logger;

    /**
     * Intentionally privatized constructor.
     */
    private LwrIgtlLogConfigurator() {

    }

    /**
     * Gets a instance of the LWR-IGTL-Project logger.
     * 
     * @return the logger.
     */
    public static LwrIgtlLogConfigurator getInstance() {
	if (null == instance) {
	    instance = new LwrIgtlLogConfigurator();
	}
	return instance;
    }

    /**
     * Sets up the loggers for the LWRopenIGTL project. Depending
     * 
     * @param smartPadLogger
     *            the Sunrise-Logging-mechanism
     * @param debugModeEnabled
     *            flag that can enable debug-logging
     * @param type
     *            the type how log records are processed.
     * 
     * @throws IOException
     *             if log-filehandler setup fails.
     */
    public void setup(final ITaskLogger smartPadLogger,
	    final boolean debugModeEnabled, final LogForwardType type)
		    throws IOException {

	logger = Logger.getLogger(LOGGERS_NAME);
	//Remove all handlers from logger
	for (Handler handler : logger.getHandlers()) {
	    logger.removeHandler(handler);
	}
	logger.setUseParentHandlers(false); // remove console handler
	logger.setLevel(Level.ALL);
	SimpleFormatter formatter = new SimpleFormatter();

	// Add smartpadLogHandler as default log handler.
	normalSmartPadHandler = new SmartPadLogHandler(smartPadLogger);
	normalSmartPadHandler.setFormatter(formatter);
	normalSmartPadHandler.setLevel(Level.ALL);
	logger.addHandler(normalSmartPadHandler); // add file handler

	// If debug mode is enabled add additional debug logger
	if (debugModeEnabled) {
	    switch (type) {

	    case File:
		smartPadLogger
			.warn("DEBUG LOGGER is enabled. All logging-output "
				+ "(all log-levels) will be "
				+ "directed to the file: " + DEBUG_LOGFILE.getAbsolutePath().toString()
				+ " additionally to the smartPad console output.");
		debugFileHandler = new FileHandler(
			DEBUG_LOGFILE.getAbsolutePath(), false);
		debugFileHandler.setLevel(Level.ALL);
		debugFileHandler.setFormatter(new XMLFormatter());
		logger.addHandler(debugFileHandler); // add file handler
		break;
	    case Network:
		smartPadLogger
			.warn("DEBUG LOGGER is enabled. All logging-output "
				+ "(all log-levels) will be "
				+ "directed to the network address: "
				+ LwrIgtlLogConfigurator.DEBUG_VIEWER_IP
				+ " with the port "
				+ LwrIgtlLogConfigurator.DEBUG_VIEWER_PORT
				+ " additionally to the smartPad console output.");

		debugSocketHandler = new SocketHandler(DEBUG_VIEWER_IP,
			DEBUG_VIEWER_PORT);
		debugSocketHandler.setLevel(Level.ALL);
		debugSocketHandler.setFormatter(new XMLFormatter());
		logger.addHandler(debugSocketHandler);
		break;
	    default:
		break;

	    }
	}

    }

    /**
     * Closes all log-handlers.
     */
    public void dispose() {

	Handler[] handlers = logger.getHandlers();

	for (Handler handler : handlers) {
	    handler.close();
	    logger.removeHandler(handler);
	    
	}
	System.out.println("Loggers disposed.");

	instance = null;
    }
}
