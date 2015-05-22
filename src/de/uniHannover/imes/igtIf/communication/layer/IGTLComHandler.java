package de.uniHannover.imes.igtIf.communication.layer;

import java.io.IOException;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

import de.uniHannover.imes.igtIf.communication.IOpenIGTLMsg;
import de.uniHannover.imes.igtIf.logging.DummyLogger;

/**
 * Handles all communication to slicer.
 *
 */
public class IGTLComHandler implements IIGTLCommunicator {

    /**
     * The port for the slicer-visualization-thread.
     */
    public static final int SLICER_VISUAL_COM_PORT = 49002;

    /**
     * The port for the slicer-control-thread.
     */
    public static final int SLICER_CONTROL_COM_PORT = 49001;

    /** Communicator for the controls. */
    private IGTLComPort controlCom;
    /** Communicator for the visualization. */
    private IGTLComPort visualCom;
    /** Logger for errors. */
    private ITaskLogger log;
    /**
     * Flag indicating if this communicator class has been initialized. Will be
     * reseted when class is disposed.
     */
    private boolean initialized = false;

    /**
     * Constructs a new communication handler for IGTL links.
     * 
     * @param logger
     *            for having logging outputs on your smartPad, can be null -> no
     *            logging will be created by this and underlying classes.
     */
    public IGTLComHandler(final ITaskLogger logger) {
	if (null != logger) {
	    log = logger;
	} else {
	    log = new DummyLogger();
	}
    }

    /**
     * Initializes the IGTL communication layer with it's two connections. After
     * calling this init method all operations provided via the interface
     * {@linkplain IIGTLCommunicator} are possible.
     * 
     * @param timeoutControlCom
     *            the initial socket timeout in milliseconds for the control
     *            channel.
     * @param timeoutVisualCom
     *            the initial socket timeout in milliseconds for the
     *            visualization channel.
     */
    public final void init(final int timeoutControlCom,
	    final int timeoutVisualCom) {
	init(SLICER_CONTROL_COM_PORT, timeoutControlCom,
		SLICER_VISUAL_COM_PORT, timeoutVisualCom);
    }

    /**
     * Initializes the IGTL communication layer with it's two connections. After
     * calling this init method all operations provided via the interface
     * {@linkplain IIGTLCommunicator} are possible.
     * 
     * @param timeoutControlCom
     *            the initial socket timeout in milliseconds for the control
     *            channel.
     * @param timeoutVisualCom
     *            the initial socket timeout in milliseconds for the
     *            visualization channel.
     * @param controlPort
     *            the port of the control channel.
     * @param visualPort
     *            the port of the visualization channel.
     */
    public final void init(final int controlPort, final int timeoutControlCom,
	    final int visualPort, final int timeoutVisualCom) {
	/* Connect all communicators and start communication */
	controlCom = new IGTLComPort(controlPort, timeoutControlCom, log);
	log.info("Control channel connected ( " + controlCom.getInetAddress()
		+ ", " + controlCom.getPort() + ")");
	visualCom = new IGTLComPort(visualPort, timeoutVisualCom, log);
	log.info("Visualization channel connected ( "
		+ visualCom.getInetAddress() + ", " + visualCom.getPort() + ")");
	try {
	    controlCom.setup();
	    visualCom.setup();
	} catch (IOException e) {
	    log.error("Setup of a communication channel failed.", e);
	    throw new IllegalStateException(
		    "Communication failed to initialize.", e);
	}

	initialized = true;
    }

    /**
     * Closes all communication channels. After executing this method another
     * initialization is needed.
     */
    public final void dispose() {
	Exception overall = null;
	if (null != controlCom) {
	    try {
		controlCom.dispose();
	    } catch (IOException e) {
		log.error(
			"Disposing of a control communication channel failed.",
			e);
		overall = e;
	    }
	}
	if (null != visualCom) {
	    try {
		visualCom.dispose();
	    } catch (IOException e) {
		log.error(
			"Disposing of a visualization communication channel failed.",
			e);
		overall = e;
	    }
	}
	if (null != overall) {
	    throw new IllegalStateException(
		    "Dispose of communication channels failed", overall);
	}

    }

    @Override
    public final synchronized void sendMsg(final IOpenIGTLMsg msg,
	    final Channel channel) {
	checkInit();
	try {
	    switch (channel) {
	    case CONTROL:
		controlCom.sendMsg(msg);
		break;
	    case VISUAL:
		visualCom.sendMsg(msg);
		break;
	    default:
		throw new IllegalStateException("Illegal channel selected.");
	    }
	} catch (IOException e) {
	    throw new IllegalStateException(
		    "Communication has a failure. Closing the communcation.", e);
	}

    }

    @Override
    public final synchronized IOpenIGTLMsg receiveMsg() {
	checkInit();
	try {
	    return controlCom.receiveMsg();
	} catch (IOException e) {
	    throw new IllegalStateException(
		    "Communication has a failure. Closing the communcation.", e);
	}

    }

    /**
     * Checks if operation is allowed (if class is properly initialized).
     */
    private void checkInit() {
	if (!initialized) {
	    throw new IllegalStateException(
		    "The communication channels are not initialized.");
	}
    }

    /**
     * Gets this class as interface.
     * 
     * @return this class as interface.
     */
    public final IIGTLCommunicator getInterface() {
	return (IIGTLCommunicator) this;
    }

}
