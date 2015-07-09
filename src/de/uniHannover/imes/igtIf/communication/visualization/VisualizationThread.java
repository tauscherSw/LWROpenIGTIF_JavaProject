/*=========================================================================

  Program:   VisualizationThread
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
package de.uniHannover.imes.igtIf.communication.visualization;

import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;
import de.uniHannover.imes.igtIf.communication.IGTLComPort;
import de.uniHannover.imes.igtIf.communication.control.CommunicationDataProvider;
import de.uniHannover.imes.igtIf.communication.control.RobotDataSet;
import de.uniHannover.imes.igtIf.logging.LwrIgtlLogConfigurator;
import de.uniHannover.imes.igtIf.util.KinematicsLwrUtil;
import de.uniHannover.imes.igtIf.util.SleepUtil;
import de.uniHannover.imes.igtIf.util.StatisticalTimer;

/**
 * This Class for the Communication with a Visualization system using the
 * opnIGTLink protocol is based on the igtlink4j class developed at the WPI.
 * 
 * @see SimpleStateExample
 */
public class VisualizationThread extends Thread {

    // **************************Constants**********************/
    /** The port for the slicer-visualization-thread. */
    public static final int SLICER_VISUAL_COM_PORT = 49002;
    /**
     * The cycle time of the slicer-visualization-thread in milliseconds.
     */
    private static final int SLICER_VISUAL_CYLCETIME_MS = 25;

    /**
     * Enum for the type of date requested from the visualization Software
     * (Image space, robot base COF, joint space).
     */
    public static enum VisualIFDatatypes {
	/** type of robot data is in imagespace. */
	IMAGESPACE,
	/** type of robot data is in jointspace. */
	JOINTSPACE,
	/** type of robot data is in cartesian space. */
	ROBOTBASE
    }

    // **************************Flags**************************/
    /**
     * Flag indicating if conditional part of the thread's work is executed. Can
     * be set via the corresponding setter.
     */
    private boolean condWorkActive = false;

    // **************************Components*********************/
    /**
     * Current visualization data.
     */
    private VisualSenderData currentDataSet;

    /**
     * Semaphore for locking the access to the cyclic data and cyclic config
     * object.
     */
    private Object cyclicDataLock;

    /**
     * The current configuration of the sender.
     */
    private VisualSenderConfig currentSenderConfig;
    /**
     * Logging mechanism provided by jdk. In case if debug flag is active, all
     * logging output will be directed to a logfile. Otherwise logging output
     * will be displayed on the smartpad.
     */
    private Logger logger = Logger
	    .getLogger(LwrIgtlLogConfigurator.LOGGERS_NAME);

    /** IGTL communication for visualization data. */
    private IGTLComPort port;

    /** Statistical timer for main loop. */
    private StatisticalTimer timer;

    /**
     * Provider for command data received via another openIGTL channel.
     */
    private CommunicationDataProvider internalDataProvider;

    // *************************Parameters**********************/
    /** Timespan in ms the thread sleeps after periodic work. */
    private int sleepTime = SLICER_VISUAL_CYLCETIME_MS;

    // *************************Constructors********************/
    /**
     * Constructor, which initializes this thread and its components.
     * 
     * @param comDataProvider
     *            provider for command data received via another IGTL channel.
     * @throws IOException
     *             when setup of communication fails.
     */
    public VisualizationThread(final CommunicationDataProvider comDataProvider)
	    throws IOException {

	/* Process arguments */
	if (null == comDataProvider) {
	    throw new IllegalArgumentException("Arument "
		    + CommunicationDataProvider.class.getSimpleName()
		    + " is null.");

	}
	internalDataProvider = comDataProvider;

	/* Construct all components */
	port = new IGTLComPort(SLICER_VISUAL_COM_PORT, 0);
	timer = new StatisticalTimer(SLICER_VISUAL_CYLCETIME_MS);
	cyclicDataLock = new Object();
	setSenderConfiguration(null, false);

    }

    // ***************************Methods***********************/
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
	    logger.info(this.getClass().getSimpleName()
		    + " is setting up its communication.");
	    setup();

	    // Main loop until thread is interrupted from the outside
	    logger.info(this.getClass().getSimpleName()
		    + " is entering its main loop.");
	    while (!this.isInterrupted()) {
		// Statistics
		timer.loopBegin();
		// used for cyclic sleep method.
		long startTimeStamp = (long) (System.nanoTime());
		try {
		    // Do all working stuff here
		    conditionalWork();
		    work();

		    // and fell asleep
		    SleepUtil.cyclicSleep(startTimeStamp, 2, sleepTime);

		} catch (RuntimeException e) {
		    /*
		     * Catches all exceptions, which occur during thread
		     * execution except Interrupted exceptions. These caught
		     * exceptions are thrown again in the finally block and have
		     * to be handled via an external Uncaught exception handler.
		     */
		    logger.log(Level.SEVERE, "Runtime exception detected in "
			    + this.getClass().getSimpleName(), e);
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
	// nothing programmed yet
    }

    /**
     * Sends visualization data only if flag is activated.
     * 
     * @throws IOException
     *             when sending fails.
     */
    private void conditionalWork() throws IOException {
	if (getCondWork()) {
	    sendTransformation(currentDataSet, currentSenderConfig);

	}
    }

    /**
     * Setter for activation of sending of visualization data via IGTL by this
     * thread.
     * 
     * @param setting
     *            true means enabling the sender false disables it.
     */
    public final void setCondWork(final boolean setting) {
	if (getCondWork() == false && setting == true) {
	    logger.info("Sending of visualization data was enabled.");
	}
	if (getCondWork() == true && setting == false) {
	    logger.info("Sending of visualization data was disabled.");
	}
	condWorkActive = setting;
    }

    /**
     * Getter for the flag which indicates if conditional part is actively
     * executed by this thread.
     * 
     * @return true if activated otherwise false.
     */
    public final boolean getCondWork() {
	return condWorkActive;
    }

    /**
     * Disposes all components of this thread. Should be called after thread is
     * interrupted and its main while loop ended.
     */
    private void dispose() {
	if (null != port) {
	    try {
		port.dispose();
		logger.info(this.getClass().getSimpleName()
			+ " was properly disposed.");
	    } catch (IOException e) {
		logger.log(Level.SEVERE,
			"Disposing of a igtl communication channel failed.", e);
	    }
	}

	if (null != timer) {
	    logger.info("Final statistics: \n" + timer.getOverallStatistics());
	}
    }

    /**
     * Sending the Cartesian or Joint position of the robot.
     * 
     * @param data
     *            data to be send.
     * @param config
     *            the current configuration of the data to be send.
     * @throws IOException
     *             when sending fails
     */
    private void sendTransformation(final VisualSenderData data,
	    final VisualSenderConfig config) throws IOException {

	float[] transformTmp = new float[12];
	transformTmp[9] = (float) data.getCartPoseRobotBase().getTranslation()
		.getX();
	transformTmp[10] = (float) data.getCartPoseRobotBase().getTranslation()
		.getY();
	transformTmp[11] = (float) data.getCartPoseRobotBase().getTranslation()
		.getZ();

	// Check if external tcp forces have to be send also.
	if (config.getSendTcpForceFlag()) {

	    double theta = 0;
	    double phi = 0;
	    theta = -Math.asin(data.getTcpForce().normalize().getX());
	    phi = Math.atan2(data.getTcpForce().normalize().getY(), data
		    .getTcpForce().normalize().getZ());

	    Rotation rot = Rotation.ofRad(0, 0, theta).compose(
		    Rotation.ofRad(0, phi, 0));
	    for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
		    transformTmp[i + 3 * j] = (float) (data.getTcpForce()
			    .length() * rot.getMatrix().get(i, j));
		}
	    }

	    port.sendIGTLTransform("TCPForce", transformTmp);

	}

	// Checking what data type was requested
	if (config.getRequestedDatatype().name()
		.contentEquals((VisualIFDatatypes.JOINTSPACE.name()))) {

	    MatrixTransformation[] directKinematics = KinematicsLwrUtil
		    .calcDirectKinematic(data.getJntPos());
	    for (int njoint = 0; njoint < 9; njoint++) {
		if (njoint == 8) {
		    transformTmp[9] = (float) data.getCartPoseRobotBase()
			    .getTranslation().getX();
		    transformTmp[10] = (float) data.getCartPoseRobotBase()
			    .getTranslation().getY();
		    transformTmp[11] = (float) data.getCartPoseRobotBase()
			    .getTranslation().getZ();
		    for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
			    transformTmp[i + 3 * j] = (float) data
				    .getCartPoseRobotBase().getRotationMatrix()
				    .get(i, j);
			}
		    }
		    port.sendIGTLTransform("T_EE", transformTmp);

		} else {
		    transformTmp[9] = (float) directKinematics[njoint]
			    .getTranslation().getX();
		    transformTmp[10] = (float) directKinematics[njoint]
			    .getTranslation().getY();
		    transformTmp[11] = (float) directKinematics[njoint]
			    .getTranslation().getZ();
		    for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
			    transformTmp[i + 3 * j] = (float) directKinematics[njoint]
				    .getRotationMatrix().get(i, j);
			}
		    }
		    port.sendIGTLTransform("T_" + 0 + (njoint + 1),
			    transformTmp);
		}

	    }

	} else if (config.getRequestedDatatype().name()
		.contentEquals((VisualIFDatatypes.IMAGESPACE.name()))) {
	    final MatrixTransformation T = data.getCartPoseTcpExternalBase();
	    transformTmp[9] = (float) T.getTranslation().getX();
	    transformTmp[10] = (float) T.getTranslation().getY();
	    transformTmp[11] = (float) T.getTranslation().getZ();
	    for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
		    transformTmp[i + 3 * j] = (float) T.getRotationMatrix()
			    .get(i, j);
		}
	    }
	    port.sendIGTLTransform("T_EE", transformTmp);

	} else { // if the robot base pose was requested the current position in
		 // robot space is send
	    for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
		    transformTmp[i + 3 * j] = (float) data
			    .getCartPoseRobotBase().getRotationMatrix()
			    .get(i, j);
		}
	    }
	    port.sendIGTLTransform("T_EE", transformTmp);
	}

    }

    /**
     * Updates the data send to slicer.
     */
    public final void updateData() {

	/*
	 * This section updates the data, which is send to slicer by this
	 * thread.
	 */

	synchronized (cyclicDataLock) {

	    // Check if current data set was generated yet.
	    if (currentDataSet == null) {
		currentDataSet = new VisualSenderData();
	    }

	    // read external transformation (for example transformation to an
	    // image base coordinate system.
	    final MatrixTransformation trafoExternalBase = internalDataProvider
		    .getCurrentCmdPacket().getTrafo();

	    internalDataProvider.readNewRobotData();
	    RobotDataSet dataSet = internalDataProvider.getCurRobotDataSet();
	    // Construct current data set.
	    if (null == trafoExternalBase) // no external trafo was received
	    {
		currentDataSet.setData(dataSet.getCurJntPose(),
			dataSet.getTcpForce(), dataSet.getCurPose(),
			dataSet.getCurPose());
	    } else {
		currentDataSet.setData(dataSet.getCurJntPose(),
			dataSet.getTcpForce(), dataSet.getCurPose(),
			trafoExternalBase.compose(dataSet.getCurPose()));
	    }

	}
    }

    /**
     * Sets the current configuration of the sender.
     * 
     * @param datatype
     *            the desired datatype of visualization data, which is send to
     *            slicer. Set to null if old datatype should be used.
     * @param sendTcpForce
     *            set flag to true if tcp force should be send to slicer also.
     * 
     */
    public final void setSenderConfiguration(VisualIFDatatypes datatype,
	    final boolean sendTcpForce) {

	synchronized (cyclicDataLock) {

	    // Default initialization
	    if (currentSenderConfig == null) {
		currentSenderConfig = new VisualSenderConfig();
		currentSenderConfig
			.setData(VisualIFDatatypes.JOINTSPACE, false);
	    }

	    if (datatype == null) {
		currentSenderConfig.setData(
			currentSenderConfig.getRequestedDatatype(),
			sendTcpForce);

	    }

	}

    }
}
