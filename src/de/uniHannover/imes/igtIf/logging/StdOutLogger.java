package de.uniHannover.imes.igtIf.logging;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

/**
 * Logger according to the robotics-API interface {@link ITaskLogger}. This
 * logger just prints all messages and exceptions to System.out or System.err
 * output.
 * 
 */
public class StdOutLogger implements ITaskLogger {

    @Override
    public final void error(final String msg) {
	System.err.println(msg);

    }

    @Override
    public final void error(final String msg, final Throwable thrown) {
	System.err.println(msg + " " + thrown.getMessage());

    }

    @Override
    public final void fine(final String msg) {
	System.out.println(msg);

    }

    @Override
    public final void info(final String msg) {
	System.out.println(msg);

    }

    @Override
    public final void warn(final String msg) {
	System.out.println(msg);

    }

    @Override
    public final void warn(final String msg, final Throwable thrown) {
	System.out.println(msg + " " + thrown.getMessage());

    }

}
