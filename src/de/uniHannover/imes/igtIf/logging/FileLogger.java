package de.uniHannover.imes.igtIf.logging;

import java.io.File;
import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

/**
 * This class is used for logging into a file. It is especially helpfull if high
 * frequent logging output will slow down the igtl loop runtime, when logging is
 * sent to be displayed on the smartPad of the lwrIiwa controller.
 *
 */
public class FileLogger implements ITaskLogger {

    /**
     * Name for the current logfile if the unparametrized constructor is
     * choosen.
     */
    private static final String TMP_LOGFILE_NAME = FileLogger.class
	    .getSimpleName() + ".log";
    /** The current logging file. */
    private File curLogFile;
    /** the file handler for the logfile. */
    private FileHandler logFileHandler;
    /** the logging object. */
    private Logger logger;

    /**
     * Privatized constructor.
     * 
     */
    private FileLogger() {

    }

    /**
     * Creates a new File in Java's IO-Temp directory for logging. Old logs will
     * be overwritten. Only one file is generated.
     * 
     * @param name
     *            Name of the logger.
     * @throws IllegalStateException
     *             if there are IO problems opening the logfiles, or if a
     *             security manager exists and if the caller does not have
     *             LoggingPermission("control").
     */
    public FileLogger(final String name) {
	// check arguments
	if (null == name) {
	    throw new NullPointerException("Arugment name is null");
	}

	// Initialize members
	curLogFile = new File(System.getProperty("java.io.tmpdir")
		+ TMP_LOGFILE_NAME);
	logger = Logger.getLogger(name);

	// prepare logfile
	try {
	    setup(false);
	} catch (SecurityException e) {
	    throw new IllegalStateException(e);
	} catch (IOException e) {
	    throw new IllegalStateException(e);
	}
    }

    /**
     * Creates a logger which directs all logging output to a given logfile.
     * 
     * @param name
     *            Name of the logger.
     * @param logFile
     *            the file, the logging output should be written in.
     * @param append
     *            set to true if logging output should be appended to the
     *            content of the given file.
     * @throws IllegalStateException
     *             if there are IO problems opening the logfiles, or if a
     *             security manager exists and if the caller does not have
     *             LoggingPermission("control").
     */
    public FileLogger(final String name, final File logFile,
	    final boolean append) {
	// Check arguments
	if (null == name) {
	    throw new NullPointerException("Arugment name is null");
	}
	if (!logFile.isFile()) {
	    throw new IllegalArgumentException("Argument "
		    + logFile.getAbsolutePath() + " is no file.");
	}

	// Initialize members
	curLogFile = logFile;
	logger = Logger.getLogger(name);

	// prepare logfile
	try {
	    setup(append);
	} catch (SecurityException e) {
	    throw new IllegalStateException(e);
	} catch (IOException e) {
	    throw new IllegalStateException(e);
	}

    }

    /**
     * Sets up the loggers environment.
     * 
     * @param append
     *            set to true if logging output should be appended to the
     *            content of the given file.
     * @throws SecurityException
     *             if a security manager exists and if the caller does not have
     *             LoggingPermission("control").
     * @throws IOException
     *             if there are IO problems opening the logfiles.
     */
    private void setup(final boolean append) throws IOException {
	logFileHandler = new FileHandler(curLogFile.getAbsolutePath(), append);
	logger.addHandler(logFileHandler);
	SimpleFormatter formatter = new SimpleFormatter();
	logFileHandler.setFormatter(formatter);
    }

    @Override
    public final void error(final String arg0) {
	logger.log(Level.SEVERE, arg0);

    }

    @Override
    public final void error(final String arg0, final Throwable arg1) {
	logger.log(Level.SEVERE, arg0, arg1);

    }

    @Override
    public final void fine(final String arg0) {
	logger.log(Level.FINE, arg0);

    }

    @Override
    public final void info(final String arg0) {
	logger.log(Level.INFO, arg0);

    }

    @Override
    public final void warn(final String arg0) {
	logger.log(Level.WARNING, arg0);

    }

    @Override
    public final void warn(final String arg0, final Throwable arg1) {
	logger.log(Level.WARNING, arg0, arg1);

    }
}
