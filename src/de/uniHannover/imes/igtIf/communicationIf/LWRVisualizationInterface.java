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
package de.uniHannover.imes.igtIf.communicationIf;

import java.io.IOException;
import java.io.OutputStream;
import java.math.BigInteger;
import java.net.ServerSocket;
import java.nio.ByteBuffer;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

import javax.net.ServerSocketFactory;

import openIGTLink.swig.IGTLheader;
import openIGTLink.swig.IGTLtransform;
import openIGTLink.swig.igtl_header;
import openIGTLink.swig.ByteArr;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;

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
     * Represents the d parameter of the Denavit-Hartenberg robot
     * representation. It is the distance between on the perpendicular line of
     * two joints in millimeters.
     */
    private static final double[] DH_D_PARAMETER_LWRIIWA7 = new double []{
	160, 180, 180, 220, 180, 220, 80, 50
    };

    /**
     * The maximum number of allowed connection errors via openIGTlink.
     */
    private static final int MAX_ALLOWED_CONNECTION_ERROR = 100;

    /**
     * Load SWIG igtlutil library (Default Library folder is
     * "..\OpenIGTLinkLib\swig\"
     */
    static {

	System.load("C:/KRC/ApplicationServer/Git/"
		+ "IGTBasicStateMachine/OpenIGTLinkLib/SWIGigtlutil.dll");
    }

    /**
     * Time step for statistic timing of the Visualization Interface thread.
     */
    private OneTimeStep aStep;

    /**
     * Working copy of the Current Cartesian position in robot base coordinate
     * system of the robot.
     */
    private MatrixTransformation cartPose = null;

    // access this field.
    /**
     * Current Cartesian position in robot base coordinate system of the robot.
     */
    public MatrixTransformation cartPose_StateM = null; // TODO design failure

    /**
     * This integer is set to true if an connection error occurs.
     */
    private int connectionErr = 0;;

    /**
     * current client status. Intitialized as disconnected state.
     */
    private ClientStatus currentStatus = ClientStatus.DISCONNECTED; // TODO
								    // unused
								    // field

    /**
     * cycle time of the visualization interface thread. Default value is 25 ms.
     */
    public int cycleTime = 25; // TODO design failure other threads access this
			       // field.; 

    /**
     * Current selected data type to be send to the robot. Initialized as
     * datatype Robsotbase.
     */
    public VisualIFDatatypes datatype = VisualIFDatatypes.ROBOTBASE; // TODO
								     // design
								     // failure
								     // other
								     // threads
								     // access
								     // this
								     // field.

    /**
     * Flag to indicate if the Debug Information should be printed or not.
     */
    public boolean debugInfoFlag = false; // TODO design failure other threads
					  // access this field.
					 /**
     * Error Message Handler which takes care of the time consuming Error output
     * in a separate thread.
     */
    private IGTMessageHandler errHandler;
				      /**
     * Flag to indicate if an Error occurred during the last cycle.
     */
    private boolean errorFlag = false; // TODO unused field
							/**
     * Working copy of the Current Joint6 position of the robot.
     */
    private JointPosition jntPose = null;

    /**
     * Current Joint positions.
     */
    public JointPosition jntPoseStateM = null; // TODO design failure other
					       // threads access this field.

    /**
     * OpenIGTLink Client socket - socket of the connected Client.
     */
    private java.net.Socket openIGTClient = null;

    /**
     * openIGTLink visualization server socket.
     */
    private ServerSocket openIGTServer;

    /**
     * Output stream for sending the currant transformation or joint angles to
     * visualization software.
     */
    private OutputStream outstr;

    /**
     * Port number for the communication with visualization software e.g. 3D
     * Slicer. Possible ports 49001 - 49005
     */
    public int port = StateMachineApplication.SLICER_VISUAL_COM_PORT; // TODO
								      // design
								      // failure
								      // other
								      // threads
								      // access
								      // this
								      // field.

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
     * Flag to indicate if force at the tool center point should be sent or not.
     */
    public boolean sendTcpForce = false; // TODO design failure other threads
					 // access this field.

    /**
     * Vector containing the force estimated at the tool center point by the
     * internal torque sensors.
     */
    public Vector TCPForce; // TODO design failure other threads access this
			    // field.

    /**
     * Working copy of the transformation from robot base coordinate system to
     * image space coordinate system.
     */
    public MatrixTransformation tImgBase = MatrixTransformation.IDENTITY; // TODO
									  // design
									  // failure
									  // other
									  // threads
									  // access
									  // this
									  // field.

    // other threads access
							// this field.
    /**
     * Transformation from robot base coordinate system to image space
     * coordinate system.
     */
    public MatrixTransformation tImgBaseStateM = null; // TODO design failure
						       // other threads access
						       // this field.

    /**
     * Array of Matrix Transformation used to save the 8 Transformation from the
     * robot base coordinate frame to the nth coordinate system.
     */
    private MatrixTransformation[] trafoMatrixArray = new MatrixTransformation[8];

    /**
     * Flag to indicate if the Visualization interface is set active or not.
     */
    public boolean visualActive = false; // TODO design failure other threads

    // access this field.
    /**
     * Flag to indicate if the Visualization interface is running or if the
     * thread is stopped.
     */
    public boolean visualRun = false; // TODO design failure other threads
    
    /**
     * Semaphore for secure access to the shared variables.
     */
    public Semaphore visualSema = new Semaphore(1, true); // TODO design failure
							  // other threads
							  // access this field.
    
    /**
     * Statistic Timer for the Visualization Interface Thread.
     */
    public StatisticTimer visualTiming = new StatisticTimer(); // TODO design
							       // failure other
							       // threads access
							       // this field.
    

    /**
     * Constructor, which initializes this thread as a deamon.
     */
    public LWRVisualizationInterface() {
	setDaemon(true);
    }

    /**
     * In this function the homogeneous Matrix-Transformation for each Joint is
     * calculated from the set of denavit hartenberg parameter.
     * @param q The joint position for calculation.
     */
    private void calcDirectKinematic(final JointPosition q) {
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
	trafoMatrixArray[0] = trafoBaseToJoint1;
	trafoMatrixArray[1] = trafoMatrixArray[0].compose(trafoJoint1ToJoint2);
	trafoMatrixArray[2] = trafoMatrixArray[1].compose(trafoJoint2ToJoint3);
	trafoMatrixArray[3] = trafoMatrixArray[2].compose(trafoJoint3ToJoint4);
	trafoMatrixArray[4] = trafoMatrixArray[3].compose(trafoJoint4ToJoint5);
	trafoMatrixArray[5] = trafoMatrixArray[4].compose(trafoJoint5ToJoint6);
	trafoMatrixArray[6] = trafoMatrixArray[5].compose(trafoJoint6ToJoint7);
	trafoMatrixArray[7] = trafoMatrixArray[6]
		.compose(trafoJoint7ToEndeffector);

    }

    // TODO duplicate code same method can be found in LWRStateMachineInterface
    /**
     * Starts the listening server on the defined port.
     * 
     * @throws IOException
     *             when connection to openIGT server fails.
     */
    private void connectServer() throws IOException {
	stopServer();
	try {
	    ServerSocketFactory serverSocketFactory = ServerSocketFactory
		    .getDefault();
	    openIGTServer = serverSocketFactory.createServerSocket(this.port);
	    openIGTServer.setReuseAddress(true);
	    System.out.println("Visualization interface server socket "
		    + "succesfully created (port " + this.port + ")");

	} catch (IOException e) {
	    System.out
		    .println("Could not Connect to Visualization interface server");
	    throw e;
	}
    }

    /**
     * Disposes the openIGT connection by closing the connection and setting the
     * corresponding objects to null.
     */
    public final void finalize() {
	if (openIGTServer != null) {
	    try {
		openIGTServer.close();
		openIGTServer = null;
		openIGTClient.close();
		openIGTClient = null;
		System.out.println("Visualization interface server stopped");
	    } catch (IOException e) {
		e.printStackTrace();
		//TODO exception concept.
	    }
	}

    }

    // TODO duplicate code. This method already exists in
    // state-machine-interface.
    /**
     * Function to restart the IGTLink Server and reinitialize the connection.
     */
    private void restartIGTServer() {

	errorFlag = true;
	try {
	    errHandler.messageSemaphore.tryAcquire(2, TimeUnit.MILLISECONDS);
	    errHandler.errorMessage = 
		    "StateMachineIF: Lost Connection to Client. Try to reconnect...";
	    errHandler.messageSemaphore.release();
	} catch (InterruptedException e) {
	    //TODO exception concept.
	}
	stopServer();
	try {
	    // Set up server
	    connectServer();
	    visualRun = true;
	    openIGTClient = openIGTServer.accept();
	    openIGTClient.setTcpNoDelay(true);
	    openIGTClient.setSoTimeout(1 * cycleTime);
	    this.outstr = openIGTClient.getOutputStream();
	    this.currentStatus = ClientStatus.CONNECTED;
	    errHandler.errorMessage = "Visual interface client connected ( "
		    + openIGTClient.getInetAddress() + ", "
		    + openIGTClient.getPort() + ")";
	    connectionErr = 0;

	} catch (Exception e) {
	    errHandler.errorMessage = 
		    "Couldn't connect to visualisation interface server!";

	}

    }

    /**
     * Main function of the Visualization Interface. In this function the server
     * is initialized and a packet handler is started. In a loop with a cycle
     * time of 20 ms the new Command String is received and the Acknowledgment
     * String send.
     **/
    public void run() {

	errHandler = new IGTMessageHandler();
	errHandler.setPriority(2);
	errHandler.sendername = "Visualization Interface:";
	errHandler.debugInfos = debugInfoFlag;
	errHandler.start();

	// Initializing the Communication with the Visualization Software
	try {
	    // Set up server
	    connectServer();
	    visualRun = true;
	    visualActive = true;
	    openIGTClient = openIGTServer.accept();
	    openIGTClient.setTcpNoDelay(true);
	    openIGTClient.setSoTimeout(10 * cycleTime); //TODO @Sebastian unknown value.
	    this.outstr = openIGTClient.getOutputStream();
	    this.currentStatus = ClientStatus.CONNECTED;
	    System.out.println("Visualization interface client connected ( "
		    + openIGTClient.getInetAddress() + ", "
		    + openIGTClient.getPort() + ")");

	} catch (Exception e) {
	    // TODO exception concept.
	    errorFlag = true;
	    errHandler.errorMessage = 
		    "Couldn't connect to Visualization interface server!";
	}

	while (visualRun) {
	    long startTimeStamp = (long) (System.nanoTime());
	    aStep = visualTiming.newTimeStep();
	    if (visualActive) {
		// Get new data from State machine
		try {
		    visualSema.acquire();
		    jntPose = jntPoseStateM;
		    cartPose = cartPose_StateM;
		    poseUidTmp = poseUid;
		    if (datatype.equals(VisualIFDatatypes.IMAGESPACE)) {
			tImgBase = tImgBaseStateM;
		    }
		    visualSema.release();
		    // Send the transform to Visualization

		} catch (InterruptedException e) {
		    errorFlag = true;
		    errHandler.errorMessage = "Unable to Acquire Visual Semaphore";
		}

		if (!openIGTClient.isClosed() 
			&& connectionErr < MAX_ALLOWED_CONNECTION_ERROR) {
		    sendTransformation(cartPose, jntPose);
		} else {
		    restartIGTServer();
		}

	    }
	    if (poseUidTmp == poseUidTmpOld) {
		errorFlag = true;
		errHandler.errorMessage = 
			"Visual IF: Getting Old Data from State Machine Thread";
		poseUidOldCount++;
	    }
	    poseUidTmpOld = poseUidTmp;

	    // Set the Module in Sleep mode for stability enhancement
	    try {
		    StateMachineApplication.cyclicSleep(startTimeStamp, 2, cycleTime);
		} catch (InterruptedException e) {
		    errorFlag = true;
		    errHandler.errorMessage = "Visual Thread Sleep failed!!";
		    //TODO exception concept.
		}
	    }
	    aStep.end();
	    //TODO define following used constants as class constants.
	    if (visualTiming.getMaxTimeMillis() > (double) 3 * cycleTime
		    || visualTiming.getMeanTimeMillis() > (double) 2
			    * cycleTime) {
		errorFlag = true;
		errHandler.errorMessage = 
			"VisualIF: Warning bad communication quality!";

	    }
	}

    


    //TODO duplicate code, same method in stateMachineInterface class.
    /**
     * Sends bytes.
     * @param bytes the bytes to be send.
     * @throws IOException when sending fails.
     */
    public final synchronized void sendBytes(final byte[] bytes) throws IOException {
	outstr.write(bytes);
	outstr.flush();
    }

    /**
     * In this function the transform message is packed using the SWIGGED
     * igtl_util classes and send to the openIGTClient by calling the
     * sendMessage function.
     * 
     * @param DeviceName
     *            - Device Name of the open IGTLink Transform message send to
     *            the visualization software
     * @param transform
     *            - the transformation to be send
     */

    public boolean SendIGTLTransform(String DeviceName, float[] transform) {
	byte[] bodyByte = new byte[IGTLtransform.IGTL_TRANSFORM_SIZE];
	byte[] headerByte = new byte[IGTLheader.IGTL_HEADER_SIZE];
	igtl_header header = new igtl_header();
	boolean check = false;
	header.setVersion(IGTLheader.IGTL_HEADER_VERSION);
	header.setBody_size((BigInteger
		.valueOf(IGTLtransform.IGTL_TRANSFORM_SIZE)));
	header.setName("TRANSFORM");
	header.setDevice_name(DeviceName); /* Device name */
	header.setTimestamp(BigInteger.valueOf(System.nanoTime()));
	// BodyBuffer = ByteBuffer.wrap(BodyByte);
	ByteBuffer bodyBuffer = ByteBuffer
		.allocate(IGTLtransform.IGTL_TRANSFORM_SIZE);
	final int size = 12; //TODO define all communication constants in seperate class.
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
	    sendBytes(headerByte);
	    sendBytes(bodyByte);
	    check = true;
	} catch (IOException e) {
	    check = false;
	}
	return check;

    }

    //TODO method's calculations should be moved to utility class.
    /**
     * Sending the Cartesian or Joint position of the robot.
     * 
     * @param transformCurrentPose
     *            the current position of the robot
     * @param curJntPose the current joint position.
     */
    private void sendTransformation(
	    final MatrixTransformation transformCurrentPose,
	    final JointPosition curJntPose) {

	float[] transformTmp = new float[12];
	transformTmp[9] = (float) transformCurrentPose.getTranslation().getX();
	transformTmp[10] = (float) transformCurrentPose.getTranslation().getY();
	transformTmp[11] = (float) transformCurrentPose.getTranslation().getZ();

	if (sendTcpForce) {
	    transformTmp[9] = (float) transformCurrentPose.getTranslation().getX();
	    transformTmp[10] = (float) transformCurrentPose.getTranslation().getY();
	    transformTmp[11] = (float) transformCurrentPose.getTranslation().getZ();
	    double theta = 0;
	    double phi = 0;
	    theta = -Math.asin(TCPForce.normalize().getX());
	    phi = Math.atan2(TCPForce.normalize().getY(), TCPForce.normalize()
		    .getZ());

	    Rotation rot = Rotation.ofRad(0, 0, theta).compose(
		    Rotation.ofRad(0, phi, 0));
	    for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
		    transformTmp[i + 3 * j] = (float) (TCPForce.length() * rot
			    .getMatrix().get(i, j));
		}
	    }

	    SendIGTLTransform("TCPForce", transformTmp);

	}

	// Checking what data type was requested
	if (datatype.name()
		.contentEquals((VisualIFDatatypes.JOINTSPACE.name()))) { // if
									 // joint
									 // space
									 // data
									 // was
									 // requested
									 // use
									 // the
									 // first
									 // seven
									 // values
									 // for
									 // the
									 // joint
									 // angles

	    calcDirectKinematic(jntPose);
	    for (int njoint = 0; njoint < 9; njoint++) {
		if (njoint == 8) {
		    transformTmp[9] = (float) transformCurrentPose.getTranslation().getX();
		    transformTmp[10] = (float) transformCurrentPose.getTranslation().getY();
		    transformTmp[11] = (float) transformCurrentPose.getTranslation().getZ();
		    for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
			    transformTmp[i + 3 * j] = (float) transformCurrentPose
				    .getRotationMatrix().get(i, j);
			}
		    }
		    SendIGTLTransform("T_EE", transformTmp);

		} else {
		    transformTmp[9] = (float) trafoMatrixArray[njoint]
			    .getTranslation().getX();
		    transformTmp[10] = (float) trafoMatrixArray[njoint]
			    .getTranslation().getY();
		    transformTmp[11] = (float) trafoMatrixArray[njoint]
			    .getTranslation().getZ();
		    for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
			    transformTmp[i + 3 * j] = (float) trafoMatrixArray[njoint]
				    .getRotationMatrix().get(i, j);
			}
		    }
		    SendIGTLTransform("T_" + 0 + (njoint + 1), transformTmp);
		}

	    }

	} else if (datatype.name().contentEquals(
		(VisualIFDatatypes.IMAGESPACE.name()))) {// if imagespace data
							 // was requested the
							 // current robot
							 // position in image
							 // space is calculated
							 // and send
	    MatrixTransformation T = tImgBase.compose(transformCurrentPose);
	    transformTmp[9] = (float) T.getTranslation().getX();
	    transformTmp[10] = (float) T.getTranslation().getY();
	    transformTmp[11] = (float) T.getTranslation().getZ();
	    for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
		    transformTmp[i + 3 * j] = (float) T.getRotationMatrix().get(i, j);
		}
	    }
	    SendIGTLTransform("T_EE", transformTmp);

	} else { // if the robot base pose was requested the current position in
		// robot space is send
	    for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
		    transformTmp[i + 3 * j] = (float) transformCurrentPose.getRotationMatrix()
			    .get(i, j);
		}
	    }
	    SendIGTLTransform("T_EE", transformTmp);
	}

    }

    /**
     * Stops the listening OpenIGTLink server.
     */
    private void stopServer() {
	if (openIGTServer != null) {
	    try {
		openIGTServer.close();
		openIGTServer = null;
		openIGTClient.close();
		openIGTClient = null;
		System.out.println("Visualization interface server stopped");
	    } catch (IOException e) {
		e.printStackTrace();
		// TODO exception concept.
	    }
	}
    }

}
