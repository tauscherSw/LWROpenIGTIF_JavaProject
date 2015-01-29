/*=========================================================================

  Program:   LWRStateMachineInterface
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher. Institute of Mechatronics System, Leibniz Universitaet Hannover. All rights reserved.

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

package de.uniHannover.imes.igtlf.communication.control;

import java.util.concurrent.Semaphore;
import java.io.IOException;
import java.io.InputStream;
import java.math.BigInteger;
import java.nio.ByteBuffer;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;
import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine.OpenIGTLinkErrorCode;
import de.uniHannover.imes.igtlf.communication.IGTLCommunicator;
import de.uniHannover.imes.igtlf.communication.IGTLMessage;
import de.uniHannover.imes.igtlf.logging.DummyLogger;
import openIGTLink.swig.ByteArr;
import openIGTLink.swig.IGTLheader;
import openIGTLink.swig.IGTLstring;
import openIGTLink.swig.igtl_header;

/**
 * This Class for the Communication with a control system using the opnIGTLink
 * protocol is based on the igtlink4j class developed at the WPI. For Further
 * information see WPI website. The LWRStateMachineInterface Thread is operated
 * with a 20ms cycle. An Example of the use of this class for the communication
 * with a State Control Software e.g. 3D Slicer see imesStateApllication.
 * According of the data type of the OpenIGTMessage the data is packed and send
 * to the state control. Supported data types are "STATUS", "STRING" and
 * "TRANSFORM". Later on "POINT" will be supported.
 * 
 * @author Sebastian Tauscher
 * @see SimpleStateExample
 */
public class LWRStateMachineInterface extends Thread {

    /**
     * Enum for the current client status {CONNECTED, DISCONNECTED }possible
     * client states.
     */
    private static enum ClientStatus {

	/** the client is connected. */
	CONNECTED,
	/** the client is disconnected. */
	DISCONNECTED
    }

    /**
     * The port for the slicer-control-thread.
     */
    public static final int SLICER_CONTROL_COM_PORT = 49001;

    /** Default String encoding for the OpenIGTLink String data type. */
    private static final int DEFAULT_STRING_ENCODING = 3;

    /**
     * The maximum allowed connection error for udp / tcp communication.
     */
    private static final int MAX_ALLOWED_CONNECTION_ERR = 100;

    /**
     * Maximum allowed number of repitive received equal uids.
     */
    private static final int MAX_EQUAL_UIDS = 4;

    /** Number of elements of a rotational matrix. */
    private static final int SIZE_OF_ROTATION = 9;

    /** Number of elements of a translation vector. */
    private static final int SIZE_OF_TRANS = 3;

    /**
     * Default cycle time for this main loop in milliseconds.
     */
    private static final int CYCLE_TIME_DEFAULT = 20;

    /**
     * Load SWIG igtlutil library (Default Library folder is
     * "..\OpenIGTLinkLib\swig\"
     */
    static {
	System.load("C:/KRC/ApplicationServer/Git/IGTBasicStateMachine"
		+ "/OpenIGTLinkLib/SWIGigtlutil.dll");
    }

    /**
     * Acknowledgement OpenIGTLink Message working copy.
     */
    private String ackMsg;


    /**
     * Object handles IGTL communication.
     */
     private IGTLCommunicator communicator;

    /**
     * Acknowledgement OpenIGTLink Message used for data transfer to the state
     * machine default message is "IDLE;".
     */
    public String ackStateM = "IDLE;"; // TODO design failure other threads
				       // access this field.

    /**
     * Time step for statistic timing of the State Machine Interface thread.
     */
    private OneTimeStep aStep;
    /**
     * Command OpenIGTLink Message used for data transfer to the state machine
     * default message is "IDLE;".
     */
    public String CMD_StateM = "IDLE;"; // TODO design failure other threads
					// access this field.

    /**
     * Command OpenIGTLink Message working copy.
     */
    private String cmdMsg = "IDLE;";

    /**
     * This integer is set to true if an connection error occurs.
     */
    private int connectionErr = 0;;

    /**
     * Flag to indicate if the communication interface is running or not.
     */
    public boolean comRunning = false; // TODO design failure other threads
				       // access this field.
    /**
     * Semaphore for save reading and writing of the the variables e.g.
     * CMD_StateM.
     */
    public Semaphore controlSemaphore = new Semaphore(1, true); // TODO design
								// failure other
								// threads
								// access this
								// field.
    /**
     * current status of the client status. It is initialized as disconnected
     * state.
     */
    private ClientStatus currentStatus = ClientStatus.DISCONNECTED; // TODO

    /**
     * Flag to indicate if the Debug Information should be printed or not.
     */
    public boolean debugInfos = false; // TODO design failure other threads
    /**
     * Error code.
     */
    public OpenIGTLinkErrorCode ErrorCode = OpenIGTLinkErrorCode.Ok; // TODO
								     // design
								     // failure
								     // other
								     // threads
								     // access
								     // this
								     // field.

    /**
     * error flag to indicate if an error occurred.
     */
    private boolean ErrorFlag = false; // TODO unused error flag

    /**
     * String containing the data type of the received OpenIGTLink message.
     */
    public String IGTLdatatype = "STRING"; // TODO design failure other threads
					   // access this field.

    /**
     * input stream of the socket communication.
     */
    private InputStream instr;

    /**
     * cycle time of the state control interface thread. Default value is 50 ms.
     */
    public int millisectoSleep = 50; // TODO design failure other threads access
				     // this field.

    /**
     * Statistic Timer for the State Machine Interface Thread.
     */
    public StatisticTimer SMtiming = new StatisticTimer(); // TODO design
							   // failure other
							   // threads access
							   // this field.
    /**
     * Transformation matrix from image space to robot base coordinate system..
     */
    public MatrixTransformation transformImageRobot; // TODO design failure
						     // other threads access
						     // this field.

    // access this field.
    /**
     * Flag to indicate if the transform from image space to robot base
     * coordinate is received or not.
     */
    public boolean transformReceived = false; // TODO design failure other
					      // threads access this field.

    // unused
    // error
    // flag
    /**
     * Current unified Identification number.
     */
    public long UID = 0; // TODO design failure other threads access this field.

    /**
     * Current unified Identification number (working copy).
     */
    private long UID_local = 0; // TODO design failure other threads access this
				// field.
    /**
     * Old current unified identification number.
     */
    private long UID_old = 0; // TODO design failure other threads access this

    /**
     * delay loops between receiving and sending a packet with the same UID
     * (should be 0).
     */
    private long uidDelay = 0;

    /**
     * Number of missed UID numbers in total.
     */
    public long UIDmiss = 0; // TODO design failure other threads access this
			     // field.

    /**
     * Number loops getting the same UID number (in a row).
     */
    private int uidRepeat = 0;

    /**
     * Number of maximun repetitions in a row.
     */
    public int UIDrepeat_max = 0; // TODO design failure other threads access
				  // this field.

    // field.
    /**
     * Number of detected repetitions of the unified identification number.
     */
    public int UIDrepeatNum = 0; // TODO design failure other threads access
				 // this field.

    /**
     * Logging mechanism object.
     */
    private ITaskLogger log;

    /**
     * cycle time of the state machine interface thread. Default value is 25 ms.
     */
    private int cycleTime;

    /**
     * Constructor, which initializes this thread as a daemon.
     */
    public LWRStateMachineInterface() {
	init(new DummyLogger(), CYCLE_TIME_DEFAULT);
    }

    /**
     * Constructor, which initializes this thread as a daemon.
     * 
     * @param cycleTimeMs
     *            the desired cycle time for the main loop in milliseconds.
     */
    public LWRStateMachineInterface(final int cycleTimeMs) {
	init(new DummyLogger(), cycleTimeMs);
    }

    /**
     * Constructor, which initializes this thread as a daemon.
     * 
     * @param cycleTimeMs
     *            the desired cycle time for the main loop in milliseconds.
     * @param logger
     *            an external logger which collects the logging output of this
     *            class.
     */
    public LWRStateMachineInterface(final int cycleTimeMs,
	    final ITaskLogger logger) {
	init(logger, cycleTimeMs);
    }

    /**
     * Initializes this class according to the parameters which are set before
     * in the different constructors.
     * 
     * @param logger
     *            the task logger.
     * @param cycletime
     *            the desired cycle time in ms.
     */
    private void init(final ITaskLogger logger, final int cycletime) {
	log = logger;
	cycleTime = cycletime;
	setDaemon(true);
	communicator = new IGTLCommunicator(SLICER_CONTROL_COM_PORT, cycleTime);
    }

    // /**
    // * Starts the listening server on the defined port.
    // *
    // * @throws IOException
    // * when connection to the port fails.
    // */
    // private void connectServer() throws IOException {
    // stopServer();
    // try {
    // ServerSocketFactory serverSocketFactory = ServerSocketFactory
    // .getDefault();
    // openIGTServer = serverSocketFactory.createServerSocket(this.port);
    // openIGTServer.setReuseAddress(true);
    // log.info("State machine interface server Socket succesfully "
    // + "created (port " + this.port + ")");
    // } catch (IOException e) {
    // log.error("Could not Connect to port :" + this.port + ")", e);
    // throw e; // TODO exception concept.
    // }
    // }

    // /**
    // * Closes the connection to the server and the client.
    // */
    // public final void finalize() {
    // if (openIGTServer != null) {
    // try {
    // openIGTServer.close();
    // openIGTServer = null;
    // openIGTClient.close();
    // openIGTClient = null;
    // log.info("State machine interface server stopped");
    // } catch (IOException e) {
    // log.error(
    // "Could not disconnect from state machine interface server",
    // e);
    // e.printStackTrace(); // TODO exception concept.
    // }
    // }
    //
    // };

    /**
     * In this function a message from the Client Socket is received and
     * according to the OpenIGTLink datatype the recieved data is saved in the
     * member variable CMDmessage. If the data type is neither Transform nor
     * String an ErrorMessage is created. (Error Message not used yet...)
     * 
     * @throws IOException
     *             when the received data has the wrong datatype or is
     *             incomplete.
     */
    private void receiveMessage() throws IOException {
	int retRead = 0;
	byte[] headerByte = new byte[IGTLheader.IGTL_HEADER_SIZE];
	boolean receivedNewDataFlag = false;
	String messageType = null;
	String uidString = "";
	final long uidMax = Long.MAX_VALUE;
	int bodySize = 0;
	String deviceName = null;
	byte[] bodyBytes = null;
	if (UID_old == uidMax) {
	    UID_old = -1;
	}
	UID_old = UID_local;

	while (!receivedNewDataFlag /* && instr.available()>0 */) {
	    retRead = instr.read(headerByte);
	    int enddata = 0;
	    byte[] tmpName = new byte[IGTLheader.IGTL_HEADER_TYPE_SIZE];
	    int k = 0;
	    for (int i = 2; i < 2 + IGTLheader.IGTL_HEADER_TYPE_SIZE
		    && enddata == 0; i++) {
		tmpName[k] = headerByte[i];
		if (headerByte[i] == 0) {
		    enddata = k;
		}
		k++;
	    }
	    messageType = new String(tmpName).substring(0, enddata);
	    IGTLdatatype = messageType;
	    byte[] tmpDeviceName = new byte[IGTLheader.IGTL_HEADER_DEVSIZE];
	    int l = 0;
	    enddata = 0;
	    final int readBeginPos = 14;
	    for (int j = readBeginPos; j < readBeginPos
		    + IGTLheader.IGTL_HEADER_DEVSIZE
		    && enddata == 0; j++) {
		tmpDeviceName[l] = headerByte[j];
		if (headerByte[j] == 0) {
		    enddata = l;
		}
		l++;
	    }
	    deviceName = new String(tmpDeviceName).substring(0, enddata);

	    final int bodySizeTmp = 8; // TODO define as class constant.
	    byte[] tmpBodySize = new byte[bodySizeTmp];
	    int m = 0;
	    final int readBeginTmpBody = 42; // TODO define as class constant.
	    for (int h = readBeginTmpBody; h < readBeginTmpBody + bodySizeTmp/*
									      * IGTLheader
									      * .
									      * IGTL_HEADER_NAME_SIZE
									      */; h++) {
		tmpBodySize[m] = headerByte[h];
		m++;
	    }

	    BigInteger bi = new BigInteger(tmpBodySize); // TODO @Sebastian
							 // rename variable.
	    bodySize = bi.intValue();

	    if (retRead > 0) {

		bodyBytes = new byte[bodySize];

		if (bodySize > 0) {
		    retRead = instr.read(bodyBytes);
		    if (retRead != bodySize) {
			// TODO @Sebastian unused code.
			// errorManager.error("ServerThread bodyBuf in ServerThread ret_read = "
			// + ret_read, new
			// Exception("Abnormal return from reading"),
			// ErrorManager.SERVERTHREAD_ABNORMAL_ANSWER);
		    }
		}
		if (messageType.equalsIgnoreCase("STRING")
			&& deviceName.split("_").length >= 2) {
		    uidString = deviceName.split("_")[1];
		    UID_local = Integer.parseInt(uidString);

		} else if (messageType.equals("TRANSFORM")) {
		    receivedNewDataFlag = true;
		} else {
		    log.error("State machine interface: Unexpected command name "
			    + "structure - expected is CMD_UID!!");
		    ErrorFlag = true;

		}
		if (UID_local > UID_old) {
		    receivedNewDataFlag = true;
		}
	    }
	}

	if (messageType.equalsIgnoreCase("STRING")) {
	    uidDelay = UID_local - UID_old;
	    if (uidDelay == 0) {
		if (uidRepeat == 0) {
		    UIDrepeatNum++;
		}
		uidRepeat++;
		if (UIDrepeat_max < uidRepeat) {
		    UIDrepeat_max = uidRepeat;
		}
		if (uidRepeat >= MAX_EQUAL_UIDS) {
		    log.error("State machine interface: UID has not changed for the "
			    + uidRepeat + ". time!! Check state control!");
		    ErrorCode = OpenIGTLinkErrorCode.HardwareOrCommunicationFailure;
		    ErrorFlag = true;
		}
	    } else if (uidDelay > 1) {
		UIDmiss = UIDmiss + uidDelay - 1;
		log.error("State machine interface: missed UID!!(miss count: "
			+ UIDmiss + ")");
		ErrorFlag = true;

	    } else if (uidDelay == 1) {
		uidRepeat = 0;
	    }

	    byte[] tmpString = new byte[bodySize
		    - IGTLstring.IGTL_STRING_HEADER_SIZE];
	    int p = 0;
	    final int readBeginPos = IGTLstring.IGTL_STRING_HEADER_SIZE;
	    for (int z = readBeginPos; z < bodySize; z++) {
		tmpString[p] = bodyBytes[z];
		p++;
	    }
	    try {
		String cmdString = new String(tmpString);
		cmdMsg = cmdString;
	    } catch (Exception e) {

		// TODO exception handling
		log.error("Couldn't generate new OpenIGTLink String Message!!");
		ErrorFlag = true;
	    }

	} else if (messageType.equalsIgnoreCase("TRANSFORM")) {
	    try {
		ByteBuffer bodyBuff = ByteBuffer.wrap(bodyBytes);
		double[] R_tmp = new double[SIZE_OF_ROTATION];
		double[] t_tmp = new double[SIZE_OF_TRANS];
		for (int i = 0; i < SIZE_OF_ROTATION + SIZE_OF_TRANS; i++) {
		    if (i < SIZE_OF_TRANS) {
			R_tmp[i] = bodyBuff.getDouble(i * Double.SIZE);
		    } else if (i >= SIZE_OF_TRANS
			    && i < SIZE_OF_ROTATION + SIZE_OF_TRANS) {
			t_tmp[i - SIZE_OF_ROTATION] = bodyBuff.getDouble(i
				* Double.SIZE);
		    }
		}
		transformImageRobot = MatrixTransformation.of(Vector.of(
			t_tmp[0], t_tmp[1], t_tmp[2]), Matrix.ofRowFirst(
			R_tmp[0], R_tmp[1], R_tmp[2], R_tmp[3], R_tmp[4],
			R_tmp[5], R_tmp[6], R_tmp[7], R_tmp[8]));
		IGTLdatatype = "TRANSFORM";
		log.error("Transform to Image space succesfully received:"
			+ transformImageRobot);
		transformReceived = true;

	    } catch (Exception e) {
		log.error("Couldn't generate new OpenIGTLink Transform Message!!");
		ErrorFlag = true;
	    }

	} else {
	    log.error("State machine interface: Unexpected Data type received!!");
	    ErrorFlag = true;
	}

	UID_old = UID_local;

    }

    // /**
    // * Function to restart the IGTLink Server and reinitialize the connection.
    // * This function is used if the connection to the state control client got
    // * lost.
    // */
    // private void restartIGTServer() {
    //
    // log.error("StateMachineIF: Lost Connection to Client. Try to reconnect...");
    // stopServer();
    // try {
    // // Set up server
    // connectServer();
    // comRunning = true;
    // openIGTClient = openIGTServer.accept();
    // openIGTClient.setTcpNoDelay(true);
    // openIGTClient.setSoTimeout(millisectoSleep);
    // this.outstr = openIGTClient.getOutputStream();
    // this.instr = openIGTClient.getInputStream();
    // this.currentStatus = ClientStatus.CONNECTED;
    // log.error("State machine interface client connected ( "
    // + openIGTClient.getInetAddress() + ", "
    // + openIGTClient.getPort() + ")");
    // connectionErr = 0;
    // ErrorCode = OpenIGTLinkErrorCode.Ok;
    // } catch (Exception e) {
    // log.error("Couldn't connect to state machine interface server!");
    // ErrorCode = OpenIGTLinkErrorCode.HardwareOrCommunicationFailure;
    // }
    //
    // }

    /**
     * main/run method function of the State control Interface. In this function
     * the server is initialized and a packet handler is started. In a loop with
     * a cycle time of 20 ms the new Command String is received and the
     * Acknowledgment String send to the state control (e.g. 3d Slicer).
     **/
    public final void run() {

	// Initializing the Communication with Slicer
	try {
	    communicator.setUpCommunication();
	    log.info("State machine interface client connected ( "
		    + communicator.getInetAddress() + ", "
		    + communicator.getPort() + ")");

	} catch (IOException e1) {
	    log.error("Initial set up of communicator failed.", e1);
	}

	comRunning = true; // TODO Tobi

	this.currentStatus = ClientStatus.CONNECTED; // TODO Tobi

	// Sending first Acknowledgment String to start the cyclic
	// communication
	try {

	    controlSemaphore.acquire();
	    ackMsg = ackStateM;
	    controlSemaphore.release();
	} catch (InterruptedException e) {
	    ErrorFlag = true;
	    log.error("StateMachineIF: Unable to Acquire Control Semaphore", e);
	}
	try {
	    if (!communicator.isClosed()) {
		sendIGTStringMessage(ackMsg + UID_local + ";");
	    }
	} catch (Exception e1) {
	    ErrorFlag = true;
	    log.error("StateMachineIF: Couldn't Send ACk data");
	}

	// Entering Loop for Communication - the loop is stopped if ControlRun
	// is set to false
	while (comRunning) {
	    // Starting Time
	    long startTimeStamp = (long) (System.nanoTime());

	    aStep = SMtiming.newTimeStep();

	    if (!communicator.isClosed()
		    && connectionErr < MAX_ALLOWED_CONNECTION_ERR) {
		ErrorFlag = false;
		try {

		    // Receive Message from State Control
		    receiveMessage();

		    // Write data into the public String CMD_StateM
		    try {
			controlSemaphore.acquire();
			CMD_StateM = cmdMsg;
			UID = UID_local;
			controlSemaphore.release();
		    } catch (InterruptedException e) {
			ErrorFlag = true;
			log.error("Interrupted during waiting for "
				+ controlSemaphore.toString() + ".", e);

		    }
		    connectionErr = 0;
		} catch (IOException e1) {
		    ErrorFlag = true;
		    log.error("StateMachineIF: Receive data timeout!!");
		    connectionErr++;
		}
	    } else { // If there is an connection error stop listening server
		     // and restart the server
		try {
		    communicator.restart();
		} catch (IOException e) {
		    log.error("Could not establish IGTL connection", e);

		}
	    }

	    try {
		Thread.sleep((long) millisectoSleep / 2 + 1);
	    } catch (InterruptedException e) {
		// TODO exception concept.
		ErrorFlag = true;
		log.error("StateMachineIF: Failed thread sleep!", e);
	    }

	    try {
		controlSemaphore.acquire();
		ackMsg = ackStateM;
		controlSemaphore.release();
	    } catch (InterruptedException e) {
		// TODO exception concept.
		ErrorFlag = true;
		log.error(
			"StateMachineIF: Unable to Acquire Control Semaphore",
			e);
	    }
	    if (!communicator.isClosed()
		    && connectionErr < MAX_ALLOWED_CONNECTION_ERR) {
		try {
		    sendIGTStringMessage(ackMsg + UID_local + ";");
		    connectionErr = 0;
		} catch (Exception e1) {
		    connectionErr++;
		    ErrorFlag = true;
		    log.error("StateMachineIF: Couldn't Send ACk data");
		}
	    } else {
		try {
		    communicator.restart();
		} catch (IOException e) {
		    log.error("Cannot restart IGTL communicator", e);
		}
	    }

	    // Set the Module in Sleep mode for stability enhancement
	    try {
		StateMachineApplication.cyclicSleep(startTimeStamp, 2,
			millisectoSleep);
	    } catch (InterruptedException e) {
		// TODO exception concept.
		log.error("Thread Sleep failed!");
	    }

	    aStep.end();

	    /*
	     * analysis of timer statistics.
	     */
	    // TODO @Sebastian following section must be simplified
	    if (SMtiming.getMaxTimeMillis() >= 10 * millisectoSleep
		    || SMtiming.getMeanTimeMillis() >= 2 * millisectoSleep) {
		log.error("StateMachineIF: Attention! Bad communication quality "
			+ "robot changes state to Error!");
		ErrorCode = OpenIGTLinkErrorCode.HardwareOrCommunicationFailure;
	    } else if ((SMtiming.getMaxTimeMillis() > 3.0 * millisectoSleep && SMtiming
		    .getMaxTimeMillis() < 10.0 * millisectoSleep)
		    || (SMtiming.getMeanTimeMillis() > millisectoSleep + 5 && SMtiming
			    .getMeanTimeMillis() < 2 * millisectoSleep)) {
		ErrorFlag = true;
		log.error("StateMachineIF: Warning bad communication quality!");
	    }
	} // end while

	// End all communication, when run() ends.
	try {
	    communicator.dispose();
	} catch (IOException e) {
	    log.error("Closing of the IGTL connection failed", e);
	}

    }

    // /**
    // * Sends bytes.
    // *
    // * @param bytes
    // * the array to be send.
    // * @throws IOException
    // * when sending fails.
    // */
    // private synchronized void sendBytes(byte[] bytes) throws IOException {
    // outstr.write(bytes);
    // outstr.flush();
    // }

    /**
     * Sends an OpenIGTlink String message.
     * 
     * @param message
     *            the message to be send.
     * @throws Exception
     *             TODO @Sebastian define more precise exception.
     */
    private void sendIGTStringMessage(String message) throws Exception {

	IGTLMessage currentMsg;

	byte[] bodyByte = new byte[message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE];
	byte[] headerByte = new byte[IGTLheader.IGTL_HEADER_SIZE];
	igtl_header header = new igtl_header();
	header.setVersion(IGTLheader.IGTL_HEADER_VERSION);
	header.setBody_size((BigInteger.valueOf(message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE)));
	header.setName("STRING");
	header.setDevice_name("ACK"); /* Device name */
	header.setTimestamp(BigInteger.valueOf(System.nanoTime()));
	ByteBuffer bodyBuffer = ByteBuffer.allocate(message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE);
	bodyBuffer.putShort((short) DEFAULT_STRING_ENCODING);
	bodyBuffer.putShort((short) message.length());
	bodyBuffer.put(message.getBytes());

	bodyByte = bodyBuffer.array();
	ByteArr bodyArray = new ByteArr(message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE);
	for (int i = 0; i < message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE; i++) {
	    bodyArray.setitem(i, bodyByte[i]);
	}
	ByteArr headerArray = ByteArr.frompointer(IGTLheader.PackHeader(header,
		bodyArray.cast()));
	for (int i = 0; i < IGTLheader.IGTL_HEADER_SIZE; i++) {
	    headerByte[i] = (byte) headerArray.getitem(i);
	}

	try {
	    currentMsg = new IGTLMessage();
	    currentMsg.init(headerByte, bodyByte);
	    communicator.sendMsg(currentMsg);
	} catch (IOException e) {
	    log.error("Sending of bytes failed.", e);
	    // TODO exception concept.
	}

    }

    // /**
    // * Closes the IGT server and client connection.
    // *
    // */
    // private void stopServer() {
    //
    // if (openIGTServer != null) {
    // try {
    // openIGTServer.close();
    // openIGTServer = null; // TODO besser reset function
    // openIGTClient.close();
    // openIGTClient = null; // TODO besser reset function
    // System.out.println("State machine interface server stopped");
    // } catch (IOException e) {
    // log.error("Cannot close server or client connection", e);
    // // TODO exception concept.
    // }
    // }
    // }

}
