package de.uniHannover.imes.igtIf.logging;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

/**
 * Converts all logging done via {@link Logger} to a log displayed on the KUKA
 * SmartPad.
 *
 */
class SmartPadLogHandler extends Handler {

    /** Can output logs to the smartPad. */
    private ITaskLogger logger;

    /**
     * Creates a new SmartPadLogger.
     * 
     * @param extLogger
     *            the logger of the Sunrise application.
     */
    public SmartPadLogHandler(final ITaskLogger extLogger) {
	logger = extLogger;
    }

    @Override
    public void publish(final LogRecord logRecord) {
	// Skip all logRecords which are lower than the parametrized loglevel
	if (logRecord.getLevel().intValue() >= Level.INFO.intValue()) {
	    flushImmediately(logRecord);
	}
    }

    /**
     * Flushes immediately the message to the output.
     * @param rec the logrecord.
     */
    private void flushImmediately(LogRecord rec) {

	    /*
	     * Check loglevels and transfer message and further information to
	     * the SunriseLogger.
	     */
	    if (rec.getLevel().intValue() == Level.INFO.intValue()) {
		logger.info(rec.getSourceClassName() + "|"
			+ rec.getSourceMethodName() + ": " + rec.getMessage());
	    } else if (rec.getLevel().intValue() == Level.SEVERE.intValue()) {
		if (null != rec.getThrown()) {
		    logger.error(
			    rec.getSourceClassName() + "|"
				    + rec.getSourceMethodName() + ": "
				    + rec.getMessage(), rec.getThrown());
		} else {
		    logger.error(rec.getSourceClassName() + "|"
			    + rec.getSourceMethodName() + ": "
			    + rec.getMessage());
		}

	    } else if (rec.getLevel().intValue() == Level.WARNING.intValue()) {
		logger.warn(
			rec.getSourceClassName() + "|"
				+ rec.getSourceMethodName() + ": "
				+ rec.getMessage(), rec.getThrown());
	    } else {
		logger.warn(rec.getSourceClassName() + "|"
			+ rec.getSourceMethodName() + ": " + rec.getMessage());
	    }

    }

    @Override
    public void close() {
	// nothing to do

    }

	@Override
	public void flush() {
		// Intentionally empty
		
	}

}
