/*=========================================================================

  Program:   LWRStateMachineInterface
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher. Institute of Mechatronic Systems, Leibniz Universitaet Hannover. All rights reserved.

	See License.txt for more information
	
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
	OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
	EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
	PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
	NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	=========================================================================*/
package de.uniHannover.imes.igtIf.communication.control;

import java.io.IOException;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;
import de.uniHannover.imes.igtIf.communication.layer.IGTLComPort;
import de.uniHannover.imes.igtIf.communication.layer.IOpenIGTLMsg;
import de.uniHannover.imes.igtIf.logging.DummyLogger;
import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine;
import de.uniHannover.imes.igtIf.util.StatisticalTimer;

/**
 * This Class for the Communication with a control system using the openIGTLink
 * protocol is based on the igtlink4j class developed at the WPI. For Further
 * information see WPI website. The LWRStateMachineInterface Thread is operated
 * with a 20ms cycle. An Example of the use of this class for the communication
 * with a State Control Software e.g. 3D Slicer see imesStateApplication.
 * According of the data type of the OpenIGTMessage the data is packed and send
 * to the state control. Supported data types are "STATUS", "STRING" and
 * "TRANSFORM". Later on "POINT" will be supported.
 * 
 * @see SimpleStateExample
 */
public class ControlThread extends Thread {

    // **************************Constants**********************/
    /** The port for the slicer-control-thread.*/
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

    /** Logger for logging comments. */
    private ITaskLogger log;

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
     * @param extLogger
     *            an external logger which collects the logging output of this
     *            class.
     * @throws IOException
     *             when setup of communication fails.
     */
    public ControlThread(final LwrStatemachine lwrStateMachine,
	    final CommunicationDataProvider comDataProvider,
	    final ITaskLogger extLogger)
	    throws IOException {

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

	if (null == extLogger) {
	    log = new DummyLogger();
	} else {
	    log = extLogger;
	}

	/* Construct all components */
	port = new IGTLComPort(SLICER_CONTROL_COM_PORT, 0, log);
	timer = new StatisticalTimer(SLICER_CONTROL_CYLCETIME_MS);

    }

    /**
     * Executes cyclic work as can be found in the work method. Optionally
     * conditional work will be executed. Furthermore loop timing statistics are
     * collected.
     */
    public final void run() {
	// Definition of variables here´
	RuntimeException exc = null;

	try {
	    // Initialize functions here
	    setup();

	    // Main loop until thread is interrupted from the outside
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
		    StateMachineApplication.cyclicSleep(startTimeStamp, 2,
			    sleepTime);

		} catch (RuntimeException e) {
		    /*
		     * Catches all exceptions, which occur during thread
		     * execution except Interrupted exceptions. These caught
		     * exceptions are thrown again in the finally block and have
		     * to be handled via an external Uncaught exception handler.
		     */
		    log.error("Runtime exception detected in "
			    + this.getClass().getSimpleName(), e);
		    log.warn(this.getClass().getSimpleName()
			    + " will be interrupted because it detected a "
			    + "runtime exception.");

		    exc = e;
		    this.interrupt();
		} catch (InterruptedException e) {
		    log.warn(this.getClass().getSimpleName()
			    + " will be interrupted because it detected an "
			    + "interrupted exception.");

		    this.interrupt();
		}

		// Statistics
		timer.loopEnd();
		String curStatistics = timer.printOpenIGTLStatistics();
		if (null != curStatistics) {
		    log.warn(curStatistics);
		}
	    }
	} catch (Exception e) {
	    exc = new IllegalThreadStateException(e.getMessage());
	} finally {
	    // Clean up thread.
	    dispose();

	    // throw any runtime exceptions except interrupted exception.
	    if (null != exc) {
		throw exc;
	    }

	}

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

	port.setup();

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

	/*
	 * Wait for incoming messages and write it to the internal data
	 * provider. Skip invalid messages.
	 */
	IOpenIGTLMsg receivedMsg = null;

	do {
	    receivedMsg = port.receiveMsg();
	    sleep(2);

	} while (!internalDataProvider.readNewCmdMessage(receivedMsg));
	log.info("Received a new message with "
		+ (receivedMsg.getBody().length
			+ receivedMsg.getHeader().length + " bytes."));

	/*
	 * Send answer to current uid. First acquire uid from the internal data
	 * provider, then acquire the acknowledgement-message from the
	 * statemachine. Then send it via the communication port.
	 */
	long currentUid = internalDataProvider.getCurrentCmdPacket().getUid();
	String ackString = stateMachine.getAckIgtMsg() + currentUid + ";";
	port.sendIGTStringMessage(ackString);
	log.info("Message sent: " + ackString);

    }

    private void conditionalWork() {
	if (condWorkActive) {

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
	if (null != port) {
	    try {
		port.dispose();
	    } catch (IOException e) {
		log.error("Disposing of a igtl communication channel failed.",
			e);
	    }
	}
    }

}
