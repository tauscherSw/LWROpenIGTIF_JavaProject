/*=========================================================================

  Program:   ControlThread
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation
  		/Nightly/Extensions/LightWeightRobotIGT

  Portions (c) Sebastian Tauscher, Institute of Mechatronic Systems, 
  	       Leibniz Universitaet Hannover. All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
	    this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
	    this list of conditions and the following disclaimer in the 
	    documentation and/or other materials provided with the distribution.

 * Neither the name of the Insight Software Consortium nor the names of its 
	    contributors may be used to endorse or promote products derived from 
	    this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
	OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
	PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
	NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	=========================================================================*/

package de.uniHannover.imes.igtIf.communication.control;

import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

import de.uniHannover.imes.igtIf.communication.IGTLComPort;
import de.uniHannover.imes.igtIf.communication.IIGTLMsgInterface;
import de.uniHannover.imes.igtIf.logging.LwrIgtlLogConfigurator;
import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine;
import de.uniHannover.imes.igtIf.util.SleepUtil;
import de.uniHannover.imes.igtIf.util.StatisticalTimer;

/**
 * This Class for the Communication with a control system using the openIGTLink
 * protocol. It is used as a thread, which reads cyclically control commands
 * (for the operation of the statemachine) from the openIGTLink interface and
 * stores it in an internal data provider {@link CommunicationDataProvider}.
 * Furthermore it sends acknowledgements for received commands to the
 * communication partner over openIGTLink. Timing statistics for the loop
 * iteration times are collected and outputted when the thread will be
 * interrupted. This thread is operated by default with a 20ms cycle.
 * 
 *
 */
public class ControlThread extends Thread {

    // **************************Constants**********************/
    /** The port for the slicer-control-thread. */
    public static final int SLICER_CONTROL_COM_PORT = 49001;
    /**
     * The cycle time of the slicer-control-thread in milliseconds.
     */
    private static final int SLICER_CONTROL_CYLCETIME_MS = 20;
    // **************************Flags**************************/
    /**
     * Flag indicating if conditional part of the thread's work is executed. Can
     * be set via the corresponding setter.
     */
    private boolean condWorkActive = false;

    // **************************Components*********************/
    /**
     * Provider for command data received via another openIGTL channel.
     */
    private CommunicationDataProvider internalDataProvider;

    /**
     * Logging mechanism provided by jdk. In case if debug flag is active, all
     * logging output will be directed to a logfile. Otherwise logging output
     * will be displayed on the smartpad.
     */
    private Logger logger = Logger
	    .getLogger(LwrIgtlLogConfigurator.LOGGERS_NAME);

    /** IGTL communication for state machine control. */
    private IGTLComPort port;

    /** Statistical timer for main loop. */
    private StatisticalTimer timer;

    /** The state machine. */
    private LwrStatemachine stateMachine;

    // *************************Parameters**********************/
    /** Timespan in ms the thread sleeps after periodic work. */
    private int sleepTime = SLICER_CONTROL_CYLCETIME_MS;

    /**
     * Constructor, which initializes this thread and its components.
     * 
     * @param lwrStateMachine
     *            the lwr statemachine
     * @param comDataProvider
     *            provider for command data received via another IGTL channel.
     * @throws IOException
     *             when setup of communication fails.
     */
    public ControlThread(final LwrStatemachine lwrStateMachine,
	    final CommunicationDataProvider comDataProvider) throws IOException {
	logger.entering(this.getClass().getName(), "Constructor(...)");

	/* Process arguments */
	if (null == lwrStateMachine) {
	    throw new IllegalArgumentException("Arument "
		    + LwrStatemachine.class.getSimpleName() + " is null.");
	}
	stateMachine = lwrStateMachine;

	if (null == comDataProvider) {
	    throw new IllegalArgumentException("Arument "
		    + CommunicationDataProvider.class.getSimpleName()
		    + " is null.");

	}
	internalDataProvider = comDataProvider;

	/* Construct all components */
	port = new IGTLComPort(SLICER_CONTROL_COM_PORT, 0);
	timer = new StatisticalTimer(SLICER_CONTROL_CYLCETIME_MS);

	logger.exiting(this.getClass().getName(), "Constructor(...)");
    }

    /**
     * Executes cyclic work as can be found in the work method. Optionally
     * conditional work will be executed. Furthermore loop timing statistics are
     * collected.
     */
    public final void run() {
	logger.entering(this.getClass().getName(), "run()");

	// Definition of variables here´
	RuntimeException exc = null;

	try {
	    // Initialize functions here
	    setup();
	    logger.info("Initialization done");

	    // Main loop until thread is interrupted from the outside
	    logger.info("Entering main loop.");
	    while (!this.isInterrupted()) {
		// Statistics
		timer.loopBegin();
		// used for cyclic sleep method.
		long startTimeStamp = (long) (System.nanoTime());
		try {
		    // Do all working stuff here
		    work();
		    conditionalWork();

		    // and fell asleep
		    SleepUtil.cyclicSleep(startTimeStamp, 2, sleepTime);

		} catch (RuntimeException e) {
		    /*
		     * Catches all exceptions, which occur during thread
		     * execution except Interrupted exceptions. These caught
		     * exceptions are thrown again in the finally block and have
		     * to be handled via an external Uncaught exception handler.
		     */
		    logger.log(Level.SEVERE, "Runtime exception detected", e);
		    logger.warning(this.getClass().getSimpleName()
			    + " will be interrupted because it detected a "
			    + "runtime exception.");

		    exc = e;
		    this.interrupt();
		} catch (InterruptedException e) {
		    logger.warning(this.getClass().getSimpleName()
			    + " will be interrupted because it detected an "
			    + "interrupted exception.");

		    this.interrupt();
		}

		// Statistics
		timer.loopEnd();
	    }
	} catch (Exception e) {
	    exc = new IllegalThreadStateException(e.getMessage());
	} finally {
	    // Clean up thread.
	    dispose();

	    // throw any runtime exceptions except interrupted exception.
	    if (null != exc) {
		logger.exiting(this.getClass().getName(), "run()");
		throw exc;
	    }

	}
	logger.exiting(this.getClass().getName(), "run()");
    }

    /**
     * Method for adjusting the sleep time the thread sleeps after periodic
     * work.
     * 
     * @param sleepTimeMs
     *            the new desired sleep time in milliseconds.
     */
    public final void setSleepTime(final int sleepTimeMs) {
	sleepTime = sleepTimeMs;
    }

    /**
     * Alternative starting method for the thread but with integrated delay.
     * 
     * @param delayMs
     *            the delay in milliseconds.
     * @throws InterruptedException
     *             when delay will be interrupted.
     */
    public final void start(final int delayMs) throws InterruptedException {
	super.start();
	sleep(delayMs);
    }

    // **************BEGINNING OF USER-DEFINED-FUNCTIONS******************

    /**
     * Setup all components and start them immediately.
     * 
     * @throws IOException
     *             when setup of the igtl-communication fails.
     */
    private void setup() throws IOException {
	logger.entering(this.getClass().getName(), "setup()");
	port.setup();
	Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
	logger.exiting(this.getClass().getName(), "setup()");
    }

    /**
     * Can only throw runtime exceptions all others will be caught and
     * transformed as runtime-exception.
     * 
     * @throws IOException
     *             when no message can be received.
     * @throws InterruptedException
     *             when sleeping between invalid messages is interrupted.
     */
    private void work() throws IOException, InterruptedException {
	logger.entering(this.getClass().getName(), "work()");

	/*
	 * Wait for incoming messages and write it to the internal data
	 * provider. Skip invalid messages.
	 */
	IIGTLMsgInterface receivedMsg = null;

	// Try to receive a new message. If no msg is in the in-buffer a null
	// message will be returned.
	logger.info("Waiting for a new message...");
	receivedMsg = port.receiveMsg();

	// Read the new message. Null messages (no message) will be skipped.
	if (null != receivedMsg) {
	    logger.fine("Received a new message with "
		    + (receivedMsg.getBody().length
			    + receivedMsg.getHeader().length + " bytes."));

	    internalDataProvider.readNewCmdMessage(receivedMsg);

	    /*
	     * Send answer to current uid. First acquire uid from the internal
	     * data provider, then acquire the acknowledgement-message from the
	     * statemachine. Then send it via the communication port.
	     */
	    long currentUid = internalDataProvider.getCurrentCmdPacket()
		    .getUid();
	    String ackString = stateMachine.getAckIgtMsg() + currentUid + ";";
	    port.sendIGTStringMessage(ackString);
	    logger.fine("Acknowledgement sent with content: "
		    + stateMachine.getAckIgtMsg() + " for UID "
		    + internalDataProvider.getCurrentCmdPacket().getUid());
	}

	logger.exiting(this.getClass().getName(), "work()");

    }

    private void conditionalWork() {
	if (condWorkActive) {
	    // no conditionals programmed yet
	}
    }

    public void setCondWork(final boolean setting) {
	condWorkActive = setting;
    }

    /**
     * Disposes all components of this thread. Should be called after thread is
     * interrupted and its main while loop ended.
     */
    private void dispose() {
	logger.entering(this.getClass().getName(), "dispose()");
	if (null != timer) {
	    logger.fine("Final timer statistics: \n"
		    + timer.getOverallStatistics());
	}

	if (null != port) {
	    try {
		port.dispose();
		logger.info("Disposed properly.");
	    } catch (IOException e) {
		logger.log(Level.SEVERE,
			"Disposing of a igtl communication channel failed.", e);
	    }
	}

	logger.exiting(this.getClass().getName(), "dispose()");
    }

}
