package de.uniHannover.imes.igtIf.logging;

import java.io.File;
import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

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

    /** File object where debug logging output is saved in. */
    public static final File DEBUG_LOGFILE = new File(
	    System.getProperty("user.dir") + File.separator + "logs"
		    + File.separator
		    + StateMachineApplication.class.getSimpleName()
		    + "_debugLog.log");

    /** Name of the logger. */
    public static final String LOGGERS_NAME = StateMachineApplication.class
	    .getSimpleName();

    /** Singleton instance. */
    private static LwrIgtlLogConfigurator instance;

    /** The current logging file. */
    private File curLogFile;
    /** the file handler for logging output. */
    private FileHandler debugLogHandler;
    /** the handler for smartPad console output. */
    private SmartPadLogHandler smartPadLogHandler;
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
     * @param debugModeEnabled
     *            flag that can enable debug-logging
     * @param smartPadLogger
     *            the Sunrise-Logging-mechanism
     * @throws IOException
     *             if log-filehandler setup fails.
     */
    public void setup(final boolean debugModeEnabled,
	    final ITaskLogger smartPadLogger) throws IOException {
	logger = Logger.getLogger(LOGGERS_NAME);
	logger.setUseParentHandlers(false); // remove console handler

	// Add smartpadLogHandler as default log handler.
	smartPadLogHandler = new SmartPadLogHandler(smartPadLogger);
	smartPadLogHandler.setLevel(Level.INFO);
	logger.addHandler(smartPadLogHandler); // add file handler

	// If debug mode is enabled add additional debug logger
	if (debugModeEnabled) {
	    debugLogHandler = new FileHandler(curLogFile.getAbsolutePath(),
		    false);
	    debugLogHandler.setLevel(Level.ALL);
	    logger.addHandler(debugLogHandler); // add file handler
	}

	SimpleFormatter formatter = new SimpleFormatter();
	debugLogHandler.setFormatter(formatter);
	smartPadLogHandler.setFormatter(formatter);

    }

    /**
     * Closes all log-handlers.
     */
    public void dispose() {
	if (null != debugLogHandler) {
	    debugLogHandler.close();

	}
	if (null != smartPadLogHandler) {
	    smartPadLogHandler.close();
	}
    }

}
