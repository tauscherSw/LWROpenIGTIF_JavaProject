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

package de.uniHannover.imes.igtIf.communicationIf;

import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.math.BigInteger;
import java.net.ServerSocket;
import java.nio.ByteBuffer;

import javax.net.ServerSocketFactory;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;
import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine.OpenIGTLinkErrorCode;
import openIGTLink.swig.ByteArr;
import openIGTLink.swig.IGTLheader;
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
     * The maximum allowed connection error for udp / tcp communication.
     */
    private static final int MAX_ALLOWED_CONNECTION_ERR = 100;

    /**
     * Maximum allowed number of repitive received equal uids.
     */
    private static final int MAX_EQUAL_UIDS = 4;

    /**
     * Load SWIG igtlutil library (Default Library folder is
     * "..\OpenIGTLinkLib\swig\"
     */
    static {
	System.load("C:/KRC/ApplicationServer/Git/IGTBasicStateMachine"
		+ "/OpenIGTLinkLib/SWIGigtlutil.dll");
    }

    /**
     * Acknowledgement OpenIGTLink Message used for data transfer to the state
     * machine default message is "IDLE;".
     */
    public String ACK_StateM = "IDLE;"; // TODO design failure other threads
					// access this field.

    /**
     * Acknowledgement OpenIGTLink Message working copy.
     */
    private String ackMsg;

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
    public boolean ControlRun = false; // TODO design failure other threads
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
     * Error Message Handler which takes care of the time consuming Error output
     * in a separate thread.
     */
    private IGTMessageHandler errorHandler;

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
     * cycle time of the state control interface thread. Default value is 20 ms.
     */
    public int millisectoSleep = 50; // TODO design failure other threads access
				     // this field.

    /**
     * OpenIGTLink Client socket - socket of the connected Client.
     */
    private java.net.Socket openIGTClient = null;

    /**
     * Server socket for the communication with state control interface.
     */
    private ServerSocket openIGTServer;

    /**
     * output stream of the socket communication.
     */
    private OutputStream outstr;

    /**
     * port number for the communication with state control. Supported ports:
     * 49001 - 49005.
     */
    public int port = StateMachineApplication.SLICER_CONTROL_COM_PORT; // TODO
								       // duplicated
								       // code,
								       // see
								       // StateMachineApplication.

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
     * Constructor, which initializes this thread as a daemon.
     */
    public LWRStateMachineInterface() {
	setDaemon(true);
    }

    /**
     * Starts the listening server on the defined port.
     * 
     * @throws IOException
     *             when connection to the port fails.
     */
    private void connectServer() throws IOException {
	stopServer(); // TODO einbauen anstatt einfach zu stoppen (z.B. is port
		      // open())
	try {
	    ServerSocketFactory serverSocketFactory = ServerSocketFactory
		    .getDefault();
	    openIGTServer = serverSocketFactory.createServerSocket(this.port);
	    openIGTServer.setReuseAddress(true);
	    System.out
		    .println("State machine interface server Socket succesfully "
			    + "created (port " + this.port + ")");
	} catch (IOException e) {
	    System.out.println("Could not Connect to port :" + this.port + ")");
	    throw e;
	}
    }

    /**
     * Closes the connection to the server and the client.
     */
    public final void finalize() {
	if (openIGTServer != null) {
	    try {
		openIGTServer.close();
		openIGTServer = null;
		openIGTClient.close();
		openIGTClient = null;
		System.out.println("State machine interface server stopped");
	    } catch (IOException e) {
		e.printStackTrace();
	    }
	}

    };

    /**
     * In this function a message from the Client Socket is received and
     * according to the OpenIGTLink datatype the recieved data is saved in the
     * member variable CMDmessage. If the data type is neither Transform nor
     * String an ErrorMessage is created. (Error Message not used yet...)
     * 
     * @throws IOException
     */
    public void receiveMessage() throws IOException {
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
		    errorHandler.errorMessage = 
			    "State machine interface: Unexpected command name structure "
			    + "- expected is CMD_UID!!";
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
		    errorHandler.errorMessage = 
			    "State machine interface: UID has not changed for the "
			    + uidRepeat + ". time!! Check state control!";
		    ErrorCode = OpenIGTLinkErrorCode.HardwareOrCommunicationFailure;
		    ErrorFlag = true;
		}
	    } else if (uidDelay > 1) {
		UIDmiss = UIDmiss + uidDelay - 1;
		errorHandler.errorMessage = 
			"State machine interface: missed UID!!(miss count: "
			+ UIDmiss + ")";
		ErrorFlag = true;

	    } else if (uidDelay == 1) {
		uidRepeat = 0;
	    }

	    final int bodyDiff = 4; // TODO @Sebastian redefine as class
				    // constant, usage unknown.
	    byte[] tmpString = new byte[bodySize - bodyDiff];
	    int p = 0;
	    final int readBeginPos = 4;
	    for (int z = readBeginPos; z < bodySize; z++) {
		tmpString[p] = bodyBytes[z];
		p++;
	    }
	    try {
		String cmdString = new String(tmpString);
		cmdMsg = cmdString;
	    } catch (Exception e) {

		// TODO exception handling
		errorHandler.errorMessage = 
			"Couldn't generate new OpenIGTLink String Message!!";
		ErrorFlag = true;
	    }

	} else if (messageType.equalsIgnoreCase("TRANSFORM")) {
	    try {
		ByteBuffer bodyBuff = ByteBuffer.wrap(bodyBytes);
		/*
		 * TODO @Sebastian: Following section holds multiple hardcoded
		 * constants. Rename variables and define class constants for
		 * solution.
		 */
		double[] R_tmp = new double[9];
		double[] t_tmp = new double[3];
		for (int i = 0; i < 12; i++) {
		    if (i < 9) {
			R_tmp[i] = bodyBuff.getDouble(i * 8);
		    } else if (i >= 9 && i < 12) {
			t_tmp[i - 9] = bodyBuff.getDouble(i * 8);
		    }
		}
		transformImageRobot = MatrixTransformation.of(Vector.of(
			t_tmp[0], t_tmp[1], t_tmp[2]), Matrix.ofRowFirst(
			R_tmp[0], R_tmp[1], R_tmp[2], R_tmp[3], R_tmp[4],
			R_tmp[5], R_tmp[6], R_tmp[7], R_tmp[8]));
		IGTLdatatype = "TRANSFORM";
		errorHandler.errorMessage = 
			"Transform to Image space succesfully received:"
			+ transformImageRobot;
		transformReceived = true;

	    } catch (Exception e) {
		errorHandler.errorMessage = 
			"Couldn't generate new OpenIGTLink Transform Message!!";
		ErrorFlag = true;
	    }

	} else {
	    errorHandler.errorMessage = 
		    "State machine interface: Unexpected Data type received!!";
	    ErrorFlag = true;
	}

	UID_old = UID_local;

    }

    /**
     * Function to restart the IGTLink Server and reinitialize the connection.
     * This function is used if the connection to the state control client got
     * lost.
     */
    private void restartIGTServer() {

	ErrorFlag = true;
	try {
	    errorHandler.messageSemaphore.tryAcquire(2, TimeUnit.MILLISECONDS);
	    errorHandler.errorMessage = 
		    "StateMachineIF: Lost Connection to Client. Try to reconnect...";
	    errorHandler.messageSemaphore.release();
	} catch (InterruptedException e) {
	    // TODO exception concept
	    e.printStackTrace();

	}
	stopServer();
	try {
	    // Set up server
	    connectServer();
	    ControlRun = true;
	    openIGTClient = openIGTServer.accept();
	    openIGTClient.setTcpNoDelay(true);
	    openIGTClient.setSoTimeout(millisectoSleep);
	    this.outstr = openIGTClient.getOutputStream();
	    this.instr = openIGTClient.getInputStream();
	    this.currentStatus = ClientStatus.CONNECTED;
	    errorHandler.errorMessage = 
		    "State machine interface client connected ( "
		    + openIGTClient.getInetAddress()
		    + ", "
		    + openIGTClient.getPort() + ")";
	    connectionErr = 0;
	    ErrorCode = OpenIGTLinkErrorCode.Ok;
	} catch (Exception e) {
	    errorHandler.errorMessage = 
		    "Couldn't connect to state machine interface server!";
	    ErrorCode = OpenIGTLinkErrorCode.HardwareOrCommunicationFailure;
	}

    }

    /**
     * main/run method function of the State control Interface. In this function
     * the server is initialized and a packet handler is started. In a loop with
     * a cycle time of 20 ms the new Command String is received and the
     * Acknowledgment String send to the state control (e.g. 3d Slicer).
     **/
    public void run() {

	// Init the ErrorHandler
	errorHandler = new IGTMessageHandler();
	errorHandler.setPriority(2);
	errorHandler.sendername = "State Control Interface:";
	errorHandler.debugInfos = debugInfos;
	errorHandler.start();

	// Initializing the Communication with the Visualization Software
	try {
	    // Set up server
	    connectServer();
	    ControlRun = true;
	    openIGTClient = openIGTServer.accept();
	    openIGTClient.setTcpNoDelay(true);
	    openIGTClient.setSoTimeout(millisectoSleep);
	    this.outstr = openIGTClient.getOutputStream();
	    this.instr = openIGTClient.getInputStream();
	    this.currentStatus = ClientStatus.CONNECTED;
	    System.out.println("State machine interface client connected ( "
		    + openIGTClient.getInetAddress() + ", "
		    + openIGTClient.getPort() + ")");

	    // Sending first Acknowledgment String to start the cyclic
	    // communication
	    try {

		controlSemaphore.acquire();
		ackMsg = ACK_StateM;
		controlSemaphore.release();
	    } catch (InterruptedException e) {
		ErrorFlag = true;
		errorHandler.errorMessage = 
			"StateMachineIF: Unable to Acquire Control Semaphore";
	    }
	    try {
		if (!openIGTClient.isClosed()) {
		    sendIGTStringMessage(ackMsg + UID_local + ";");
		}
	    } catch (Exception e1) {
		ErrorFlag = true;
		errorHandler.errorMessage = "StateMachineIF: Couldn't Send ACk data";
	    }
	} catch (Exception e) {
	    errorHandler.errorMessage = 
		    "Couldn't connect to state machine interface server!";
	}

	// Entering Loop for Communication - the loop is stopped if ControlRun
	// is set to false
	while (ControlRun) {
	    // Starting Time
	    long startTimeStamp = (long) (System.nanoTime());

	    aStep = SMtiming.newTimeStep();

	    if (!openIGTClient.isClosed()
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
			try {
			    errorHandler.messageSemaphore.tryAcquire(2,
				    TimeUnit.MILLISECONDS);
			    errorHandler.errorMessage = 
				    "StateMachineIF:Unable to Acquire Control Semaphore";
			    errorHandler.messageSemaphore.release();
			} catch (InterruptedException e1) {
			    e1.printStackTrace();
			    // TODO exception concept.
			}

		    }
		    connectionErr = 0;
		} catch (IOException e1) {
		    ErrorFlag = true;
		    try {
			errorHandler.messageSemaphore.tryAcquire(2,
				TimeUnit.MILLISECONDS);
			errorHandler.errorMessage = 
				"StateMachineIF: Receive data timeout!!";
			errorHandler.messageSemaphore.release();
		    } catch (InterruptedException e) {
			// TODO exception concept.
			e.printStackTrace();
		    }

		    connectionErr++;
		}
	    } else { // If there is an connection error stop listening server
		     // and restart the server
		restartIGTServer();
	    }

	    try {
		Thread.sleep((long) millisectoSleep / 2 + 1);
	    } catch (InterruptedException e) {
		// TODO exception concept.
		ErrorFlag = true;
		errorHandler.errorMessage = "StateMachineIF: Failed thread sleep!";
	    }

	    try {
		controlSemaphore.acquire();
		ackMsg = ACK_StateM;
		controlSemaphore.release();
	    } catch (InterruptedException e) {
		// TODO exception concept.
		ErrorFlag = true;
		errorHandler.errorMessage = 
			"StateMachineIF: Unable to Acquire Control Semaphore";
	    }
	    if (!openIGTClient.isClosed()
		    && connectionErr < MAX_ALLOWED_CONNECTION_ERR) {
		try {
		    sendIGTStringMessage(ackMsg + UID_local + ";");
		    connectionErr = 0;
		} catch (Exception e1) {
		    connectionErr++;
		    ErrorFlag = true;
		    errorHandler.errorMessage = 
			    "StateMachineIF: Couldn't Send ACk data";
		}
	    } else {
		restartIGTServer();
	    }

	    // Set the Module in Sleep mode for stability enhancement
	    try {
		StateMachineApplication.cyclicSleep(startTimeStamp, 2,
			millisectoSleep);
	    } catch (InterruptedException e) {
		// TODO exception concept.
		errorHandler.errorMessage = "Thread Sleep failed!";
	    }

	    aStep.end();

	    /*
	     * analysis of timer statistics.
	     */
	    // TODO @Sebastian following section must be simplified
	    if (SMtiming.getMaxTimeMillis() >= 10 * millisectoSleep
		    || SMtiming.getMeanTimeMillis() >= 2 * millisectoSleep) {
		errorHandler.errorMessage = 
			"StateMachineIF: Attention! Bad communication quality "
			+ "robot changes state to Error!";
		ErrorCode = OpenIGTLinkErrorCode.HardwareOrCommunicationFailure;
	    } else if ((SMtiming.getMaxTimeMillis() > 3.0 * millisectoSleep && SMtiming
		    .getMaxTimeMillis() < 10.0 * millisectoSleep)
		    || (SMtiming.getMeanTimeMillis() > millisectoSleep + 5 && SMtiming
			    .getMeanTimeMillis() < 2 * millisectoSleep)) {
		ErrorFlag = true;
		errorHandler.errorMessage = 
			"StateMachineIF: Warning bad communication quality!";
	    }
	} // end while
    }

    /**
     * Sends bytes.
     * @param bytes the array to be send.
     * @throws IOException when sending fails.
     */
    final public synchronized void sendBytes(byte[] bytes) throws IOException {
	outstr.write(bytes);
	outstr.flush();
    }

    /**
     * Sends an OpenIGTlink String message.
     * 
     * @param message
     *            the message to be send.
     * @throws Exception
     *             TODO @Sebastian define more precise exception.
     */
    public void sendIGTStringMessage(String message) throws Exception {
	/*
	 * TODO @Sebastian Following sections has few hardcoded constants.
	 * Define them as class constants.
	 */
	byte[] bodyByte = new byte[message.length() + 4];
	byte[] headerByte = new byte[IGTLheader.IGTL_HEADER_SIZE];
	igtl_header header = new igtl_header();
	header.setVersion(IGTLheader.IGTL_HEADER_VERSION);
	header.setBody_size((BigInteger.valueOf(message.length() + 4)));
	header.setName("STRING");
	header.setDevice_name("ACK"); /* Device name */
	header.setTimestamp(BigInteger.valueOf(System.nanoTime()));
	ByteBuffer bodyBuffer = ByteBuffer.allocate(message.length() + 4);
	bodyBuffer.putShort((short) 3);
	bodyBuffer.putShort((short) message.length());
	bodyBuffer.put(message.getBytes());

	bodyByte = bodyBuffer.array();
	ByteArr bodyArray = new ByteArr(message.length() + 4);
	for (int i = 0; i < message.length() + 4; i++) {
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
	} catch (IOException e) {
	    // TODO exception concept.
	}

    }

      /**
     * Closes the IGT server and client connection.
     * 
     */
    private void stopServer() {

	if (openIGTServer != null) {
	    try {
		openIGTServer.close();
		openIGTServer = null; // TODO besser reset function
		openIGTClient.close();
		openIGTClient = null; // TODO besser reset function
		System.out.println("State machine interface server stopped");
	    } catch (IOException e) {
		// TODO was soll mit der exception gemacht werden?
		e.printStackTrace();
	    }
	}
    }

}
