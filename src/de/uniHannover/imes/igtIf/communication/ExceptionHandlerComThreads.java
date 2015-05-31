package de.uniHannover.imes.igtIf.communication;

import java.lang.Thread.UncaughtExceptionHandler;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

/**
 * Exception handler for the igtl-communication threads. It's purpose is just to
 * print all uncaugth exceptions from the according threads.
 */
public class ExceptionHandlerComThreads implements UncaughtExceptionHandler {

    // **************************Components*********************/
    /** Logger for printing the exceptions. */
    private ITaskLogger log;

    //*************************Constructors********************/
    /**
     * Creates an exception handler object.
     * 
     * @param logger
     *            the logger, used for printing the exception caught in the
     *            according threads.
     */
    public ExceptionHandlerComThreads(final ITaskLogger logger) {
	if (null == logger) {
	    throw new NullPointerException("Argument is null");
	} else {
	    log = logger;
	}
    }

    //***************************Methods***********************/
    @Override
    public final void uncaughtException(final Thread t, final Throwable e) {
	log.error("An uncaught exception occured in the thread "
		+ t.getClass().getSimpleName(), e);

    }

}
