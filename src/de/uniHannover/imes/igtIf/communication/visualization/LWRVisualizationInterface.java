/*=========================================================================

  Program:   LWRVisualizationInterface
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher, Institute of Mechatronics System, Leibniz Universitaet Hannover. All rights reserved.

	See License.txt for more information
	
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
	CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
	EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
	PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
	NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	=========================================================================*/
package de.uniHannover.imes.igtIf.communication.visualization;

import java.io.File;
import java.io.IOException;
import java.math.BigInteger;
import java.nio.ByteBuffer;

import OpenIGTLink.swig.ByteArr;
import OpenIGTLink.swig.IGTLheader;
import OpenIGTLink.swig.IGTLtransform;
import OpenIGTLink.swig.igtl_header;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;
import de.uniHannover.imes.igtIf.communication.IGTLCommunicator;
import de.uniHannover.imes.igtIf.communication.IGTLMsg;
import de.uniHannover.imes.igtIf.communication.control.CommunicationDataProvider;
import de.uniHannover.imes.igtIf.communication.control.RobotDataSet;
import de.uniHannover.imes.igtIf.logging.DummyLogger;
import de.uniHannover.imes.igtIf.util.FileSystemUtil;

/**
 * This Class for the Communication with a Visualization system using the
 * opnIGTLink protocol is based on the igtlink4j class developed at the WPI.
 * 
 * @author Sebastian Tauscher
 * @see SimpleStateExample
 */
public class LWRVisualizationInterface extends Thread {

    /**
     * Enum for the client status. Possible states are connected and
     * disconnected.
     */
    private static enum ClientStatus { // TODO duplicate enumeration
	/** the client is connected. */
	CONNECTED,
	/** the client is disconnected. */
	DISCONNECTED
    }

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

    /**
     * Default cycle time for this main loop in milliseconds.
     */
    private static final int CYCLE_TIME_DEFAULT = 25;

    /**
     * Represents the d parameter of the Denavit-Hartenberg robot
     * representation. It is the distance between on the perpendicular line of
     * two joints in millimeters.
     */
    private static final double[] DH_D_PARAMETER_LWRIIWA7 = new double[] { 160,
	    180, 180, 220, 180, 220, 80, 50 };

    /**
     * The maximum number of allowed connection errors via openIGTlink.
     */
    private static final int MAX_ALLOWED_CONNECTION_ERROR = 100;

    /**
     * The port for the slicer-visualization-thread.
     */
    public static final int SLICER_VISUAL_COM_PORT = 49002;

    /**
     * In this function the homogeneous Matrix-Transformation for each Joint is
     * calculated from the set of denavit hartenberg parameter.
     * 
     * @param q
     *            The joint position for calculation.
     * @return the composed array of {@link MatrixTransformation} to each joint
     *         center point and to the flange. All transformations are
     *         calculation in relatively to the robots base.
     */
    private static MatrixTransformation[] calcDirectKinematic(
	    final JointPosition q) {
	MatrixTransformation trafoBaseToJoint1 = MatrixTransformation.of(Vector
		.of(0, 0, DH_D_PARAMETER_LWRIIWA7[0]), Matrix.ofRowFirst(

	Math.cos(q.get(0)), -Math.sin(q.get(0)), 0, Math.sin(q.get(0)),
		Math.cos(q.get(0)), 0, 0, 0, 1));

	MatrixTransformation trafoJoint1ToJoint2 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[1]), Matrix.ofRowFirst(
			Math.cos(q.get(1)), 0, Math.sin(q.get(1)), 0, 1, 0,
			-Math.sin(q.get(1)), 0, Math.cos(q.get(1))));

	MatrixTransformation trafoJoint2ToJoint3 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[2]), Matrix.ofRowFirst(
			Math.cos(q.get(2)), -Math.sin(q.get(2)), 0,
			Math.sin(q.get(2)), Math.cos(q.get(2)), 0, 0, 0, 1));

	MatrixTransformation trafoJoint3ToJoint4 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[3]), Matrix.ofRowFirst(
			Math.cos(-q.get(3)), 0, Math.sin(-q.get(3)), 0, 1, 0,
			-Math.sin(-q.get(3)), 0, Math.cos(-q.get(3))));

	MatrixTransformation trafoJoint4ToJoint5 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[4]), Matrix.ofRowFirst(
			Math.cos(q.get(4)), -Math.sin(q.get(4)), 0,
			Math.sin(q.get(4)), Math.cos(q.get(4)), 0, 0, 0, 1));

	MatrixTransformation trafoJoint5ToJoint6 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[5]), Matrix.ofRowFirst(
			Math.cos(q.get(5)), 0, Math.sin(q.get(5)), 0, 1, 0,
			-Math.sin(q.get(5)), 0, Math.cos(q.get(5))));

	MatrixTransformation trafoJoint6ToJoint7 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[6]), Matrix.ofRowFirst(
			Math.cos(q.get(6)), -Math.sin(q.get(6)), 0,
			Math.sin(q.get(6)), Math.cos(q.get(6)), 0, 0, 0, 1));

	MatrixTransformation trafoJoint7ToEndeffector = MatrixTransformation
		.of(Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[7]),
			Matrix.IDENTITY);
	MatrixTransformation[] retVal = new MatrixTransformation[8];
	retVal[0] = trafoBaseToJoint1;
	retVal[1] = retVal[0].compose(trafoJoint1ToJoint2);
	retVal[2] = retVal[1].compose(trafoJoint2ToJoint3);
	retVal[3] = retVal[2].compose(trafoJoint3ToJoint4);
	retVal[4] = retVal[3].compose(trafoJoint4ToJoint5);
	retVal[5] = retVal[4].compose(trafoJoint5ToJoint6);
	retVal[6] = retVal[5].compose(trafoJoint6ToJoint7);
	retVal[7] = retVal[6].compose(trafoJoint7ToEndeffector);
	return retVal;

    }

    /**
     * Object handles IGTL communication.
     */
    private IGTLCommunicator communicator;

    /**
     * Semaphore for locking the access to the cyclic data and cyclic config
     * object.
     */
    private Object cyclicDataLock;

    /**
     * Time step for statistic timing of the Visualization Interface thread.
     */
    private OneTimeStep aStep;

    /**
     * This integer is set to true if an connection error occurs.
     */
    private int connectionErr = 0;

    /**
     * Current visualization data.
     */
    private VisualSenderData currentDataSet;

    /**
     * The current configuration of the sender.
     */
    private VisualSenderConfig currentSenderConfig;

    /**
     * current client status. Initialized as disconnected state.
     */
    private ClientStatus currentStatus = ClientStatus.DISCONNECTED; // TODO
								    // unused
								    // field;

    /**
     * cycle time of the visualization interface thread. Default value is 25 ms.
     */
    private int cycleTime;

    /**
     * Logger for all logging outputs of this class.
     */
    private ITaskLogger log;

    /**
     * UID of the current pose sent to the visualization software.
     */
    public int poseUid = 0;// TODO design failure other threads access this
			   // field.

    /**
     * UID of the last pose sent to the visualization software.
     */
    public int poseUidOldCount = 0;// TODO design failure other threads access
				   // this field.

    /**
     * Working copy of UID.
     */
    private int poseUidTmp = 0;

    /**
     * Working copy of old UID.
     */
    private int poseUidTmpOld = -1;

    /**
     * Provider for command data received via another openIGTL channel.
     */
    private CommunicationDataProvider comDataSink;

    /**
     * Flag to indicate if the Visualization interface is running or if the
     * thread is stopped.
     */
    private boolean visualRun = false;

    /**
     * Flag to indicate if the visualization interface is sending data.
     */
    private boolean transmitDataFlag = false;

    // /**
    // * Semaphore for secure access to the shared variables.
    // */
    // public Semaphore visualSema = new Semaphore(1, true); // TODO design
    // failure
    // // other threads
    // // access this field.

    /**
     * Statistic Timer for the Visualization Interface Thread.
     */
    public StatisticTimer visualTiming = new StatisticTimer(); // TODO design
							       // failure other
							       // threads access
							       // this field.

    /**
     * Constructor, which initializes this thread as a daemon.
     * 
     * @param comDataProvider
     *            provider for command data received via another IGTL channel.
     */
    public LWRVisualizationInterface(
	    final CommunicationDataProvider comDataProvider) {

	init(comDataProvider, new DummyLogger(), CYCLE_TIME_DEFAULT);

    }

    /**
     * Constructor, which initializes this thread as a daemon.
     * 
     * @param cycleTimeMs
     *            the desired cycle time for the main loop in ms.
     * @param comDataProvider
     *            provider for command data received via another IGTL channel.
     */
    public LWRVisualizationInterface(final int cycleTimeMs,
	    final CommunicationDataProvider comDataProvider) {
	init(comDataProvider, new DummyLogger(), cycleTimeMs);
    }

    /**
     * Constructor, which initializes this thread as a daemon.
     * 
     * @param cycleTimeMs
     *            the desired cycle time for the main loop in ms.
     * @param logger
     *            an external logger for this class.
     * @param comDataProvider
     *            provider for command data received via another IGTL channel.
     */
    public LWRVisualizationInterface(
	    final CommunicationDataProvider comDataProvider,
	    final int cycleTimeMs, final ITaskLogger logger) {
	init(comDataProvider, logger, cycleTimeMs);
    }

    /**
     * Initializes this class according to the parameters which are set before
     * in the different constructors.
     * 
     * @param comDataProvider
     *            provider for command data received via another IGTL channel.
     * @param logger
     *            the task logger.
     * @param cycletime
     *            the desired cycle time in ms.
     */
    private void init(final CommunicationDataProvider comDataProvider,
	    final ITaskLogger logger, final int cycletime) {
	comDataSink = comDataProvider;
	log = logger;
	cycleTime = cycletime;
	setDaemon(true);
	communicator = new IGTLCommunicator(SLICER_VISUAL_COM_PORT, cycleTime);
	cyclicDataLock = new Object();
    }

    // // TODO duplicate code same method can be found in
    // LWRStateMachineInterface
    // /**
    // * Starts the listening server on the defined port.
    // *
    // * @throws IOException
    // * when connection to openIGT server fails.
    // */
    // private void connectServer() throws IOException {
    // stopServer();
    // try {
    // ServerSocketFactory serverSocketFactory = ServerSocketFactory
    // .getDefault();
    // openIGTServer = serverSocketFactory.createServerSocket(this.port);
    // openIGTServer.setReuseAddress(true);
    // log.info("Visualization interface server socket "
    // + "succesfully created (port " + this.port + ")");
    //
    // } catch (IOException e) {
    // log.error("Could not Connect to Visualization interface server", e);
    // throw e; // TODO exception concept.
    // }
    // }

    /**
     * This method enables the main loop to run, and thus activates this
     * communication thread.
     * 
     * @param value
     *            set to true if cyclic communication should be enabled
     *            otherwise false.
     */
    public final void setCyclicCommunication(final boolean value) {
	visualRun = value;
    }

    /**
     * This method enables the transmission of data by the visualization
     * interface thread.
     * 
     * @param value
     *            set to true if data transmission should be enabled otherwise
     *            false.
     */
    public final void setDataTransmission(final boolean value) {
	transmitDataFlag = value;
    }

    /**
     * This method enables the transmission of data by the visualization
     * interface thread.
     */
    public final void disableDataTransmission() {
	transmitDataFlag = false;
    }

    /**
     * Getter for the flag indicating if the data transmission of this
     * visualization interface is enabled.
     * 
     * @return true if enabled otherwise false;
     */
    public final boolean isDataTransmissionEnable() {
	return transmitDataFlag;
    }

    // /**
    // * Disposes the openIGT connection by closing the connection and setting
    // the
    // * corresponding objects to null.
    // */
    // public final void finalize() {
    // if (openIGTServer != null) {
    // try {
    // openIGTServer.close();
    // openIGTServer = null;
    // openIGTClient.close();
    // openIGTClient = null;
    // log.info("Visualization interface server stopped");
    // } catch (IOException e) {
    // log.error(
    // "Could not disconnect from visualization interface server",
    // e);
    // e.printStackTrace(); // TODO exception concept.
    // }
    // }
    //
    // }

    /**
     * Getter for the current connection state of this igtl-communicator.
     * 
     * @return the current client status.
     */
    public final ClientStatus getCurState() {
	return currentStatus;
    }

    /**
     * This method evaluates if this thread is enabled to run cyclic.
     * 
     * @return true if enabled otherwise false.
     */
    public final boolean isEnabled() {
	return visualRun;
    }

    /**
     * This prevents the main loop, from being executed another time.
     */
    public final void quitCommunication() {
	visualRun = false;
    }

    // // TODO duplicate code. This method already exists in
    // // state-machine-interface.
    // /**
    // * Function to restart the IGTLink Server and reinitialize the connection.
    // */
    // private void restartIGTServer() {
    //
    // log.error("StateMachineIF: Lost Connection to Client. Try to reconnect...");
    //
    // stopServer();
    // try {
    // // Set up server
    // connectServer();
    // setCyclicCommunication(true);
    // openIGTClient = openIGTServer.accept();
    // openIGTClient.setTcpNoDelay(true);
    // openIGTClient.setSoTimeout(1 * cycleTime);
    // this.outstr = openIGTClient.getOutputStream();
    // this.currentStatus = ClientStatus.CONNECTED;
    // log.error("Visual interface client connected ( "
    // + openIGTClient.getInetAddress() + ", "
    // + openIGTClient.getPort() + ")");
    // connectionErr = 0;
    //
    // } catch (Exception e) {
    // log.error("Couldn't connect to visualisation interface server!");
    //
    // }
    //
    // }

    /**
     * Main function of the Visualization Interface. In this function the server
     * is initialized and a packet handler is started. In a loop with a cycle
     * time of 20 ms the new Command String is received and the Acknowledgment
     * String send.
     **/
    public final void run() {

	// // Initializing the Communication with the Visualization Software
	// try {
	// // Set up server
	// connectServer();
	// setCyclicCommunication(true);
	// openIGTClient = openIGTServer.accept();
	// openIGTClient.setTcpNoDelay(true);
	// openIGTClient.setSoTimeout(10 * cycleTime); // TODO @Sebastian
	// // unknown value.
	// this.outstr = openIGTClient.getOutputStream();
	// this.currentStatus = ClientStatus.CONNECTED;
	// log.info("Visualization interface client connected ( "
	// + openIGTClient.getInetAddress() + ", "
	// + openIGTClient.getPort() + ")");
	//
	// } catch (Exception e) {
	// // TODO exception concept.
	//
	// log.error("Couldn't connect to Visualization interface server!");
	// }
	try {
	    communicator.setup();
	    log.info("Visualization interface client is connected ( "
		    + communicator.getInetAddress() + ", "
		    + communicator.getPort() + ")");

	} catch (IOException e1) {
	    log.error("Initial set up of communicator failed.", e1);
	}

	// Enter the main loop
	log.info(this.getName() + " enters the main loop");
	while (isEnabled()) {
	    log.info(this.getName() + " begins its main loop");
	    long startTimeStamp = (long) (System.nanoTime());
	    aStep = visualTiming.newTimeStep();

	    if (!communicator.isClosed()
		    && connectionErr < MAX_ALLOWED_CONNECTION_ERROR
		    && isDataTransmissionEnable()) {
		synchronized (cyclicDataLock) {
		    log.info(this.getName() + " sends a transformation.");
		    sendTransformation(currentDataSet, currentSenderConfig);
		}

	    } else {
		try {
		    log.warn(this.getName()
			    + " restarts its communicator port because "
			    + "the connection was closed.");
		    communicator.restart(10 * cycleTime);
		    log.info(this.getName()
			    + " restarted successfully its communicator.");
		} catch (IOException e) {
		    log.error("Could not establish IGTL connection", e);

		}
	    }

	    // TODO @Tobias uids
	    if (poseUidTmp == poseUidTmpOld) {

		log.warn(this.getName()
			+ " receives old Data from State Machine Thread (according to the UIDs)");
		poseUidOldCount++;
	    }
	    poseUidTmpOld = poseUidTmp;

	    // Set the Module in Sleep mode for stability enhancement
	    try {
		StateMachineApplication.cyclicSleep(startTimeStamp, 2,
			cycleTime);
	    } catch (InterruptedException e) {

		log.error(this.getName() + " sleep failed!!");
		// TODO exception concept.
	    }
	    aStep.end();

	    // TODO define following used constants as class constants.
	    // TODO @Tobias getTimingStatistics() should replace the following
	    // part.
	    if (visualTiming.getMaxTimeMillis() > (double) 3 * cycleTime
		    || visualTiming.getMeanTimeMillis() > (double) 2
			    * cycleTime) {

		log.warn(this.getName() + " has bad communication quality!");

	    }
	    log.info(this.getName() + " ends its main loop");
	}// end while

	// End all communication, when run() ends.
	try {
	    log.info(this.getName() + " will be disposed.");
	    communicator.dispose();
	} catch (IOException e) {
	    log.error("Closing of the IGTL connection failed", e);
	}
    }

    // // TODO duplicate code, same method in stateMachineInterface class.
    // /**
    // * Sends bytes.
    // *
    // * @param bytes
    // * the bytes to be send.
    // * @throws IOException
    // * when sending fails.
    // */
    // public final synchronized void sendBytes(final byte[] bytes)
    // throws IOException {
    // outstr.write(bytes);
    // outstr.flush();
    // }

    /**
     * In this function the transform message is packed using the SWIGGED
     * igtl_util classes and send to the openIGTClient by calling the
     * sendMessage function.
     * 
     * @param deviceName
     *            - Device Name of the open IGTLink Transform message send to
     *            the visualization software
     * @param transform
     *            - the transformation to be send
     * @return true if sending was successful otherwise false.
     */
    public final boolean sendIGTLTransform(final String deviceName,
	    final float[] transform) {
	IGTLMsg currentMessage = new IGTLMsg();
	byte[] bodyByte = new byte[IGTLtransform.IGTL_TRANSFORM_SIZE];
	byte[] headerByte = new byte[IGTLheader.IGTL_HEADER_SIZE];
	igtl_header header = new igtl_header();
	boolean check = false;
	header.setVersion(IGTLheader.IGTL_HEADER_VERSION);
	header.setBody_size((BigInteger
		.valueOf(IGTLtransform.IGTL_TRANSFORM_SIZE)));
	header.setName("TRANSFORM");
	header.setDevice_name(deviceName); /* Device name */
	header.setTimestamp(BigInteger.valueOf(System.nanoTime()));
	// BodyBuffer = ByteBuffer.wrap(BodyByte);
	ByteBuffer bodyBuffer = ByteBuffer
		.allocate(IGTLtransform.IGTL_TRANSFORM_SIZE);
	final int size = 12; // TODO define all communication constants in
			     // seperate class.
	for (int i = 0; i < size; i++) {
	    bodyBuffer.putFloat(transform[i]);
	}

	bodyByte = bodyBuffer.array();
	ByteArr bodyArray = new ByteArr(bodyByte.length);
	for (int i = 0; i < bodyByte.length; i++) {
	    bodyArray.setitem(i, bodyByte[i]);
	}

	ByteArr headerArray = ByteArr.frompointer(IGTLheader.PackHeader(header,
		bodyArray.cast()));
	for (int i = 0; i < IGTLheader.IGTL_HEADER_SIZE; i++) {
	    headerByte[i] = (byte) headerArray.getitem(i);
	}

	try {
	    currentMessage.init(headerByte, bodyByte);
	    communicator.sendMsg(currentMessage);
	    check = true;
	} catch (IOException e) {
	    check = false;
	}
	return check;

    }

    /**
     * Sending the Cartesian or Joint position of the robot.
     * 
     * @param data
     *            data to be send.
     * @param config
     *            the current configuration of the data to be send.
     */
    private void sendTransformation(final VisualSenderData data,
	    final VisualSenderConfig config) {

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

	    sendIGTLTransform("TCPForce", transformTmp);

	}

	// Checking what data type was requested
	if (config.getRequestedDatatype().name()
		.contentEquals((VisualIFDatatypes.JOINTSPACE.name()))) {

	    MatrixTransformation[] directKinematics = calcDirectKinematic(data
		    .getJntPos());
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
		    sendIGTLTransform("T_EE", transformTmp);

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
		    sendIGTLTransform("T_" + 0 + (njoint + 1), transformTmp);
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
	    sendIGTLTransform("T_EE", transformTmp);

	} else { // if the robot base pose was requested the current position in
		 // robot space is send
	    for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
		    transformTmp[i + 3 * j] = (float) data
			    .getCartPoseRobotBase().getRotationMatrix()
			    .get(i, j);
		}
	    }
	    sendIGTLTransform("T_EE", transformTmp);
	}

    }

    /**
     * Registers an external logger to this thread. All logging output will be
     * directed to the external logger.
     * 
     * @param externalLogger
     *            the external logger object.
     */
    public final void setLogger(final ITaskLogger externalLogger) {
	log = externalLogger;
    }

    // /**
    // * Stops the listening OpenIGTLink server.
    // */
    // private void stopServer() {
    // if (openIGTServer != null) {
    // try {
    // openIGTServer.close();
    // openIGTServer = null;
    // openIGTClient.close();
    // openIGTClient = null;
    // log.info("Visualization interface server stopped");
    // } catch (IOException e) {
    // e.printStackTrace();
    // // TODO exception concept.
    // }
    // }
    // }

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
	    final MatrixTransformation trafoExternalBase = comDataSink
		    .getCurrentCmdPacket().getTrafo();

	    comDataSink.readNewRobotData();
	    RobotDataSet dataSet = comDataSink.getCurRobotDataSet();
	    // Construct current data set.
	    currentDataSet.setData(dataSet.getCurJntPose(),
		    dataSet.getTcpForce(), dataSet.getCurPose(),
		    trafoExternalBase.compose(dataSet.getCurPose()));
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

	    if (currentSenderConfig == null) {
		currentSenderConfig = new VisualSenderConfig();
	    }
	    if (datatype == null) {
		datatype = currentSenderConfig.getRequestedDatatype();

	    }
	    currentSenderConfig.setData(datatype, sendTcpForce);
	}

    }

}
