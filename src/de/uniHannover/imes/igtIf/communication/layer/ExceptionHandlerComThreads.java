package de.uniHannover.imes.igtIf.communication.layer;

import java.lang.Thread.UncaughtExceptionHandler;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

public class ExceptionHandlerComThreads implements UncaughtExceptionHandler {

	private ITaskLogger log;

	public ExceptionHandlerComThreads(final ITaskLogger logger) {
		if (null == logger) {
			throw new NullPointerException("Argument is null");
		} else {
			log = logger;
		}
	}

	@Override
	public void uncaughtException(Thread t, Throwable e) {
		log.error("An uncaught exception occured in the thread "
				+ t.getClass().getSimpleName(), e);

	}

}
