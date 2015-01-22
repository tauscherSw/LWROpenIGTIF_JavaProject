package de.uniHannover.imes.igtlf.logging;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

/**
 * Dummy logger logs nowhere, log messages are gone when logged with this
 * logger.
 */
public class DummyLogger implements ITaskLogger {

    @Override
    public void error(final String msg) {
	// intentionally do nothing

    }

    @Override
    public void error(final String msg, final Throwable thrown) {
	// intentionally do nothing

    }

    @Override
    public void fine(final String msg) {
	// intentionally do nothing

    }

    @Override
    public void info(final String msg) {
	// intentionally do nothing

    }

    @Override
    public void warn(final String msg) {
	// intentionally do nothing

    }

    @Override
    public void warn(final String msg, final Throwable thrown) {
	// intentionally do nothing

    }

}
