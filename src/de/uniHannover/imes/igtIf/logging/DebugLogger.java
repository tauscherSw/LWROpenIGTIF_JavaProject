package de.uniHannover.imes.igtIf.logging;

import java.io.File;
import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;

/**
 * This class provides a uniform way to log high frequent debug-logging output.
 * The debug logger is defined as a {@link FileLogger} in
 * {@link StateMachineApplication}.
 *
 */
public final class DebugLogger implements ITaskLogger {

    /** Debug logger name. */
    private static final String IGTIF_DEBUG_LOGGER = "DebugLogger";

    /** File object where debug logging output is saved in. */
    public static final File DEBUG_LOGFILE = new File(
	    System.getProperty("user.dir") + File.separator + "logs"
		    + File.separator + IGTIF_DEBUG_LOGGER + ".log");

    /** Instance of the logger as singleton pattern. */
    private static ITaskLogger instance;

    /** Logger for all debugging output. */
    private ITaskLogger logger;

    /** Creates a debug logger object. */
    private DebugLogger() {
	logger = new FileLogger(IGTIF_DEBUG_LOGGER, DEBUG_LOGFILE, false);
    }

    /**
     * Gives access to a debug logger.
     * 
     * @return the reference to the logger.
     * */
    public static ITaskLogger getInstance() {
	if (null == instance) {
	    instance = new DebugLogger();

	}
	return instance;
    }

    @Override
    public void error(final String arg0) {
	logger.error(arg0);

    }

    @Override
    public void error(final String arg0, final Throwable arg1) {
	logger.error(arg0, arg1);

    }

    @Override
    public void fine(final String arg0) {
	logger.fine(arg0);

    }

    @Override
    public void info(final String arg0) {
	logger.info(arg0);

    }

    @Override
    public void warn(final String arg0) {
	logger.warn(arg0);

    }

    @Override
    public void warn(final String arg0, final Throwable arg1) {
	logger.warn(arg0, arg1);

    }

}
