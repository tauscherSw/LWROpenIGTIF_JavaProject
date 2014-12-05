package de.uniHannover.imes.igtIf.interfaces;

import java.util.concurrent.Semaphore;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;

import javax.net.ServerSocketFactory;

import org.medcare.igtl.messages.OpenIGTMessage;
import org.medcare.igtl.messages.PositionMessage;
import org.medcare.igtl.messages.StatusMessage;
import org.medcare.igtl.messages.StringMessage;
import org.medcare.igtl.messages.TransformMessage;
import org.medcare.igtl.util.Header;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;

/**
 * This Class for the Communication with a control system using the opnIGTLink
 * protocol is based on the igtlink4j class developed at the WPI. For Further
 * information see WPI Homepage. The LWRStateMachineInterface Thread is operated
 * with a 20ms cycle. An Example of the use of this class for the communication
 * with a State Control Software e.g. 3D Slicer see imesStateApllication.
 * According of the datatype of the OpenIGTMessage the data is packed and send
 * to the state control. Supported datattypes are "STATUS", "STRING" and
 * "TRANSFORM". Later on "POINT" will be supported.
 * 
 * @author Sebastian Tauscher
 * @see imesStateApplication
 */
public class LWRStateMachineInterface extends Thread {
    /** One time step in the state machine. Each cycle takes one timestep.*/
    private OneTimeStep aStep;

    /**
     * Statistic Timer for the Visualization Interface Thread.
     */
    public StatisticTimer SMtiming = new StatisticTimer();

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
     * input stream of the socket communication.
     */
    private InputStream instr;

    /**
     * error flag to indicate if an error occured.
     */
    private boolean errorFlag = false;

    /**
     * Current enum for the client status {CONNECTED, DISCONNECTED }possible
     * client states.
     */
    public static enum ClientStatus {
	/** the client is connected in this state.*/
	CONNECTED, 
	/** the client is disconnected in this state.*/
	DISCONNECTED
    }; // possible client states

    /**
     * current status of the client status.
     */
    private ClientStatus currentStatus = ClientStatus.DISCONNECTED; // start as
								    // stopped
								    // status

    /**
     * Current unified Identification number.
     */
    public int uId = 0;
    /**
     * Current unified Identification number (working copy).
     */
    private int uIdLocal = 0;

    /**
     * Old current unified identification number.
     */
    public int uIdOld = 0;

    /**
     * Command OpenIGTLink Message default message is "IDLE;".
     */
    public OpenIGTMessage cmdStateMsg = new StringMessage("CMD_0000", "IDLE;");

    /**
     * Acknowledgement OpenIGTLink Message default message is "IDLE;".
     */
    public OpenIGTMessage ackStateMsg = new StringMessage("ACK_0000", "IDLE;");

    /**
     * Acknowledgement OpenIGTLink Message working copy.
     */
    private OpenIGTMessage ackMsg;

    /**
     * Command OpenIGTLink Message working copy.
     */
    private OpenIGTMessage cmdMsg = null;

    /**
     * Error Message String for error in the State machine interface.
     */
    public String errMsg;

    /**
     * in this String the last printed error message is saved to check if it is
     * error message has already been printed.
     */
    private String lastPrintedError = "";

    /**
     * Semaphore for save reading and writing the variables.
     */
    public Semaphore ctrlSema = new Semaphore(1, true);
    /**
     * cycle time of the state control interface thread. Default value is 20 ms.
     */
    private static final int CYCLE_TIME_CTRL_IF = 20;

    /**
     * port number for the communication with state control. Possible ports
     * 49001 - 49005.
     */
    public int port = 49001;

    /**
     * Flag to indicate if the communication interface is running or not.
     */
    public boolean comIfActive = false;

    /**
     * Number of missed UID numbers in total.
     */
    public int uIdMissed = 0;

    /**
     * Number loops getting the same UID number (in a row).
     */
    public int uIdRepeat = 0;

    /**
     * delay loops between receiving and sending a packet with the same UID
     * (should be 0).
     */
    public int UIDdelay = 0;

    /**
     * Starts the listening server on the defined port.
     * 
     * @param port
     *            the port for the communication with state control.
     * @throws IOException
     */
    public final void ConnectServer(int port) throws IOException {
	stopServer();
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
     * Stops the listening OpenIGTLink server.
     * 
     */
    public final void stopServer() {

	if (openIGTServer != null) {
	    try {
		openIGTServer.close();
		openIGTServer = null;
		System.out.println("State machine interface server stopped");
	    } catch (IOException e) {
		e.printStackTrace();
	    }
	}
	// socket = null;
	// currentStatus = ServerStatus.STOPPED;
    }

    /**
     * Constructor, which initializes this interface as daemon thread.
     */
    public LWRStateMachineInterface() {
	setDaemon(true);
    };

    /**
     * Initialize function of the State control Interface. In this function the
     * server is initialized and a packet handler is started. In a loop with a
     * cycle time of 20 ms the new Command String is received and the
     * Acknowledgment String send.
     * 
     * @param IP
     *            the IP address of the State Control Computer, e.g. the system
     *            where your Slicer/Matlab GUI is running.
     * @param port
     *            the port number used for the communication with the State
     *            Control Computer.
     * @return
     * @see
     **/
    public void run() {

	// Initializing the Communication with the Visualization Software
	try {
	    // Set up server
	    ConnectServer(port);
	    comIfActive = true;
	    openIGTClient = openIGTServer.accept();
	    openIGTClient.setTcpNoDelay(true);
	    this.outstr = openIGTClient.getOutputStream();
	    this.instr = openIGTClient.getInputStream();
	    this.currentStatus = ClientStatus.CONNECTED;
	    System.out.println("State machine interface client connected ( "
		    + openIGTClient.getInetAddress() + ", "
		    + openIGTClient.getPort() + ")");

	} catch (Exception e) {
	    // TODO Auto-generated catch block
	    errMsg = "Couldn't connect to state machine interface server!";
	}
	while (comIfActive) {
	    long startTimeStamp = (long) (System.nanoTime());
	    int startTimeStampNanos = (int) (System.nanoTime() 
		    - startTimeStamp * 1000000);
	    aStep = SMtiming.newTimeStep();
	    // Get new data from State machine
	    errorFlag = false;
	    try {
		if (!openIGTClient.isClosed()) {
		    receiveMessage();
		    try {
			ctrlSema.acquire();
			cmdStateMsg = cmdMsg;
			uId = uIdLocal;
			ctrlSema.release();
		    } catch (InterruptedException e) {
			errorFlag = true;
			errMsg = "StateMachineIF:Unable to Acquire Control Semaphore";
		    }
		}
	    } catch (IOException e1) {
		// TODO Automatisch generierter Erfassungsblock
		e1.printStackTrace();
	    }

	    try {
		Thread.sleep((long) 11.0);
	    } catch (InterruptedException e) {
		// TODO
		errorFlag = true;
		errMsg = "StateMachineIF: Failed thread sleep!";
	    }

	    try {
		ctrlSema.acquire();
		ackMsg = ackStateMsg;
		ctrlSema.release();
	    } catch (InterruptedException e) {
		// TODO
		errorFlag = true;
		errMsg = "StateMachineIF: Unable to Acquire Control Semaphore";
	    }
	    try {
		if (!openIGTClient.isClosed()) {
		    ackMsg.PackBody();
		    sendMessage(ackMsg);
		}
	    } catch (Exception e1) {
		// TODO Automatisch generierter Erfassungsblock
		errorFlag = true;
		errMsg = "StateMachineIF: Couldn't Send ACk data";
	    }
	    if (errorFlag) {

		if (!errMsg.equals(lastPrintedError)) {
		    System.out.println(errMsg);
		    lastPrintedError = errMsg;
		}
	    } else {
		lastPrintedError = "";
	    }
	    // Set the Module in Sleep mode for stability enhancement
	    long curTime = (long) ((System.nanoTime() 
		    - startTimeStamp) / 1000000.0);
	    int curTimeNanos = (int) ((System.nanoTime() 
		    - startTimeStampNanos) - curTime * 1000000.0);
	    if (curTime < CYCLE_TIME_CTRL_IF) {
		// ThreadUtil.milliSleep((long) Math.floor((millisectoSleep-1 -
		// curTime)));
		try {
		    Thread.sleep(CYCLE_TIME_CTRL_IF - curTime, curTimeNanos);
		} catch (InterruptedException e) {
		    // TODO Automatisch generierter Erfassungsblock
		    e.printStackTrace();
		}
	    }

	    aStep.end();
	}
    }

    /**
     * In this function a message from the Client Socket is received and
     * according to the OpenIGTLink datatype the recieved data is saved in the
     * member variable CMDmessage. If the data type is neither Transform nor
     * String an ErrorMessage is created. (Error Message not used yet...)
     * 
     * @throws IOException when the received message cannot be read.
     */
    public final void receiveMessage() throws IOException {
	int retRead = 0;
	byte[] headerBuff = new byte[Header.LENGTH];
	String messageType;
	String uIdString = "";
	retRead = instr.read(headerBuff);
	if (retRead > 0) {
	    Header header = new Header(headerBuff);
	    byte[] bodyBuf = new byte[(int) header.getBody_size()];
	    // System.out.print("ServerThread Header deviceName : " +
	    // header.getDeviceName() + " Type : " + header.getDataType() +
	    // " bodySize " + header.getBody_size() + "\n");
	    if ((int) header.getBody_size() > 0) {
		retRead = instr.read(bodyBuf);
		if (retRead != header.getBody_size()) {
		    // errorManager.error("ServerThread bodyBuf in ServerThread ret_read = "
		    // + ret_read, new
		    // Exception("Abnormal return from reading"),
		    // ErrorManager.SERVERTHREAD_ABNORMAL_ANSWER);
		}
	    }
	    // Log.debug("New Header: "+header);
	    // BytesArray b = new BytesArray();
	    // b.putBytes(bodyBuf);
	    // Log.debug("New Body: "+b);
	    messageType = header.getDataType();
	    uIdOld = uIdLocal;
	    if (2 == header.getDeviceName().split("_").length) {
		uIdString = header.getDeviceName().split("_")[1];
		uIdLocal = Integer.parseInt(uIdString);
		UIDdelay = uIdLocal - uIdOld;
		if (UIDdelay == 0) {
		    uIdRepeat++;
		    errMsg = "State machine interface: UID has not changed for the "
			    + uIdRepeat + ". time!! Check state control!";
		    errorFlag = true;
		} else if (UIDdelay > 1) {
		    uIdMissed++;
		    errMsg = "State machine interface: missed UID!!(miss count: "
			    + uIdMissed + ")";
		    errorFlag = true;

		} else if (UIDdelay == 1) {
		    uIdRepeat = 0;
		}
	    } else {
		errMsg = "State machine interface: Unexpected command name structure "
			+ "- expected is CMD_UID!!";
		errorFlag = true;

	    }
	    if (messageType.equals("STRING")) {
		try {
		    StringMessage textContent = new StringMessage(header, bodyBuf);
		    cmdMsg = textContent;
		} catch (Exception e) {
		    // TODO Automatisch generierter Erfassungsblock
		    e.printStackTrace();
		}

		String body = new String(bodyBuf, 4, bodyBuf.length - 4,
			"US-ASCII");
	    } else if (messageType.equals("TRANSFORM")) {
		try {
		    TransformMessage transform = new TransformMessage(header,
			    bodyBuf);
		    transform.Unpack();
		    double[][] rTmp = transform.getRotationMatrixArray();
		    double[] tTmp = transform.getPosition();
		    Matrix rotMatrix = Matrix
			    .ofColumnFirst(rTmp[0][0], rTmp[1][0],
				    rTmp[2][0], rTmp[0][1], rTmp[1][1],
				    rTmp[2][1], rTmp[0][2], rTmp[1][2],
				    rTmp[2][2]);

		    System.out.println("Transform:"
			    + transform.getRotationMatrixArray());
		    MatrixTransformation transformationMatrix = MatrixTransformation.of(
			    Vector.of(tTmp[0], tTmp[1], tTmp[2]), rotMatrix);
		    cmdMsg = transform;
		} catch (Exception e) {
		    // TODO Automatisch generierter Erfassungsblock
		    e.printStackTrace();
		}

	    } else {
		errMsg = "State machine interface: Unexpected Data type received!!";
		errorFlag = true;
	    }
	}

    }

    public void pushPose(String deviceName, TransformNR pose) {
	PositionMessage poseMsg = new PositionMessage(deviceName,
		pose.getPositionArray(), pose.getRotationMatrix());
	try {
	    sendMessage(poseMsg);
	} catch (Exception e) {
	    // TODO Automatisch generierter Erfassungsblock
	    e.printStackTrace();
	}
    }

    public void pushStatus(String deviceName, int code, int subCode,
	    String status) {
	StatusMessage statMsg = new StatusMessage(deviceName, code, subCode,
		status);
	try {
	    sendMessage(statMsg);
	} catch (Exception e) {
	    // TODO Automatisch generierter Erfassungsblock
	    e.printStackTrace();
	}
    }

    public void pushStatus(String deviceName, int code, int subCode,
	    String errorName, String status) {
	StatusMessage statMsg = new StatusMessage(deviceName, code, subCode,
		errorName, status);
	try {
	    sendMessage(statMsg);
	} catch (Exception e) {
	    // TODO Automatisch generierter Erfassungsblock
	    e.printStackTrace();
	}
    }

    public void pushStringMessage(String deviceName, String msg) {
	StringMessage strMsg = new StringMessage(deviceName, msg);
	try {
	    sendMessage(strMsg);
	} catch (Exception e) {
	    // TODO Automatisch generierter Erfassungsblock
	    // e.printStackTrace();
	}
    }

    public void pushTransformMessage(String deviceName, TransformNR t) {
	TransformMessage transMsg = new TransformMessage(deviceName,
		t.getPositionArray(), t.getRotationMatrixArray());
	transMsg.PackBody();
	try {
	    sendMessage(transMsg);
	} catch (Exception e) {
	    // TODO Automatisch generierter Erfassungsblock
	    e.printStackTrace();
	}
    }

    public void sendMessage(OpenIGTMessage message) throws Exception {
	// TODO Auto-generated method stub
	sendMessage(message.getHeader(), message.getBody());
	// System.out.println("Message: Header=" +
	// message.getHeader().toString() + " Body=" +
	// message.getBody().toString());
    }

    public void sendMessage(Header header, byte[] body) throws Exception {
	sendBytes(header.getBytes());
	sendBytes(body);
	// System.out.println("Sending Message: Header=" + header.toString() +
	// " Body=" + body.toString());
    }

    /***************************************************************************
     * Sends bytes
     * <p>
     * 
     * @throws IOException
     *             - Exception in I/O.
     *             <p>
     * @param bytes
     *            - byte[] array.
     **************************************************************************/
    final public synchronized void sendBytes(byte[] bytes) throws IOException {
	outstr.write(bytes);
	outstr.flush();
    }

    /***************************************************************************
     * Interrupt this thread
     **************************************************************************/

}
