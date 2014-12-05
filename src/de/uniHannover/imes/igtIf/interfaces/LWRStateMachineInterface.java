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
    private boolean ErrorFlag = false;

    /**
     * Current enum for the client status {CONNECTED, DISCONNECTED }possible
     * client states.
     */
    public static enum ClientStatus {
	CONNECTED, DISCONNECTED
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
    public int UID = 0;
    /**
     * Current unified Identification number (working copy).
     */
    private int UID_local = 0;

    /**
     * Old current unified identification number.
     */
    public int UID_old = 0;

    /**
     * Command OpenIGTLink Message default message is "IDLE;".
     */
    public OpenIGTMessage CMD_StateM = new StringMessage("CMD_0000", "IDLE;");

    /**
     * Acknowledgement OpenIGTLink Message default message is "IDLE;".
     */
    public OpenIGTMessage ACK_StateM = new StringMessage("ACK_0000", "IDLE;");

    /**
     * Acknowledgement OpenIGTLink Message working copy.
     */
    private OpenIGTMessage ACKmessage;

    /**
     * Command OpenIGTLink Message working copy.
     */
    private OpenIGTMessage CMDmessage = null;

    /**
     * Error Message String for error in the State machine interface.
     */
    public String ErrorMessage;

    /**
     * in this String the last printed error message is saved to check if it is
     * error message has already been printed.
     */
    private String LastPrintedError = "";

    /**
     * Semaphore for save reading and writing the variables.
     */
    public Semaphore ControlSemaphore = new Semaphore(1, true);
    /**
     * cycle time of the state control interface thread. Default value is 20 ms.
     */
    public int millisectoSleep = 20;

    /**
     * port number for the communication with state control. Possible ports
     * 49001 - 49005.
     */
    public int port = 49001;

    /**
     * Flag to indicate if the communication interface is running or not.
     */
    public boolean ControlRun = false;

    /**
     * Number of missed UID numbers in total.
     */
    public int UIDmiss = 0;

    /**
     * Number loops getting the same UID number (in a row).
     */
    public int UIDrepeat = 0;

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
    public void ConnectServer(int port) throws IOException {
	stopServer();
	try {
	    ServerSocketFactory serverSocketFactory = ServerSocketFactory
		    .getDefault();
	    openIGTServer = serverSocketFactory.createServerSocket(this.port);
	    openIGTServer.setReuseAddress(true);
	    System.out
		    .println("State machine interface server Socket succesfully created (port "
			    + this.port + ")");
	} catch (IOException e) {
	    System.out.println("Could not Connect to port :" + this.port + ")");
	    throw e;
	}
    }

    /**
     * Stops the listening OpenIGTLink server.
     * 
     */
    public void stopServer() {

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
	// TODO Automatisch generierter Methodenstub

	String ACKname = null;
	// Initializing the Communication with the Visualization Software
	try {
	    // Set up server
	    ConnectServer(port);
	    ControlRun = true;
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
	    ErrorMessage = "Couldn't connect to state machine interface server!";
	}
	while (ControlRun) {
	    long startTimeStamp = (long) (System.nanoTime());
	    int startTimeStamp_nanos = (int) (System.nanoTime() - startTimeStamp * 1000000);
	    aStep = SMtiming.newTimeStep();
	    // Get new data from State machine
	    ErrorFlag = false;
	    try {
		if (!openIGTClient.isClosed()) {
		    receiveMessage();
		    try {
			ControlSemaphore.acquire();
			CMD_StateM = CMDmessage;
			UID = UID_local;
			ControlSemaphore.release();
		    } catch (InterruptedException e) {
			ErrorFlag = true;
			ErrorMessage = "StateMachineIF:Unable to Acquire Control Semaphore";
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
		ErrorFlag = true;
		ErrorMessage = "StateMachineIF: Failed thread sleep!";
	    }

	    try {
		ControlSemaphore.acquire();
		ACKmessage = ACK_StateM;
		ControlSemaphore.release();
	    } catch (InterruptedException e) {
		// TODO
		ErrorFlag = true;
		ErrorMessage = "StateMachineIF: Unable to Acquire Control Semaphore";
	    }
	    try {
		if (!openIGTClient.isClosed()) {
		    ACKmessage.PackBody();
		    sendMessage(ACKmessage);
		}
	    } catch (Exception e1) {
		// TODO Automatisch generierter Erfassungsblock
		ErrorFlag = true;
		ErrorMessage = "StateMachineIF: Couldn't Send ACk data";
	    }
	    if (ErrorFlag) {

		if (!ErrorMessage.equals(LastPrintedError)) {
		    System.out.println(ErrorMessage);
		    LastPrintedError = ErrorMessage;
		}
	    } else {
		LastPrintedError = "";
	    }
	    // Set the Module in Sleep mode for stability enhancement
	    long curTime = (long) ((System.nanoTime() - startTimeStamp) / 1000000.0);
	    int curTime_nanos = (int) ((System.nanoTime() - startTimeStamp_nanos) - curTime * 1000000.0);
	    if (curTime < millisectoSleep) {
		// ThreadUtil.milliSleep((long) Math.floor((millisectoSleep-1 -
		// curTime)));
		try {
		    Thread.sleep(millisectoSleep - curTime, curTime_nanos);
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
     * @throws IOException
     */
    public void receiveMessage() throws IOException {
	int ret_read = 0;
	byte[] headerBuff = new byte[Header.LENGTH];
	String messageType;
	String UID_String = "";
	ret_read = instr.read(headerBuff);
	if (ret_read > 0) {
	    Header header = new Header(headerBuff);
	    byte[] bodyBuf = new byte[(int) header.getBody_size()];
	    // System.out.print("ServerThread Header deviceName : " +
	    // header.getDeviceName() + " Type : " + header.getDataType() +
	    // " bodySize " + header.getBody_size() + "\n");
	    if ((int) header.getBody_size() > 0) {
		ret_read = instr.read(bodyBuf);
		if (ret_read != header.getBody_size()) {
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
	    UID_old = UID_local;
	    if (2 == header.getDeviceName().split("_").length) {
		UID_String = header.getDeviceName().split("_")[1];
		UID_local = Integer.parseInt(UID_String);
		UIDdelay = UID_local - UID_old;
		if (UIDdelay == 0) {
		    UIDrepeat++;
		    ErrorMessage = "State machine interface: UID has not changed for the "
			    + UIDrepeat + ". time!! Check state control!";
		    ErrorFlag = true;
		} else if (UIDdelay > 1) {
		    UIDmiss++;
		    ErrorMessage = "State machine interface: missed UID!!(miss count: "
			    + UIDmiss + ")";
		    ErrorFlag = true;

		} else if (UIDdelay == 1) {
		    UIDrepeat = 0;
		}
	    } else {
		ErrorMessage = "State machine interface: Unexpected command name structure - expected is CMD_UID!!";
		ErrorFlag = true;

	    }
	    if (messageType.equals("STRING")) {
		try {
		    StringMessage String = new StringMessage(header, bodyBuf);
		    CMDmessage = String;
		} catch (Exception e) {
		    // TODO Automatisch generierter Erfassungsblock
		    e.printStackTrace();
		}

		String body = new String(bodyBuf, 4, bodyBuf.length - 4,
			"US-ASCII");
	    } else if (messageType.equals("TRANSFORM")) {
		try {
		    TransformMessage Transform = new TransformMessage(header,
			    bodyBuf);
		    Transform.Unpack();
		    double[][] R_tmp = Transform.getRotationMatrixArray();
		    double[] t_tmp = Transform.getPosition();
		    com.kuka.roboticsAPI.geometricModel.math.Matrix R = com.kuka.roboticsAPI.geometricModel.math.Matrix
			    .ofColumnFirst(R_tmp[0][0], R_tmp[1][0],
				    R_tmp[2][0], R_tmp[0][1], R_tmp[1][1],
				    R_tmp[2][1], R_tmp[0][2], R_tmp[1][2],
				    R_tmp[2][2]);

		    System.out.println("Transform:"
			    + Transform.getRotationMatrixArray());
		    MatrixTransformation T = MatrixTransformation.of(
			    Vector.of(t_tmp[0], t_tmp[1], t_tmp[2]), R);
		    CMDmessage = Transform;
		} catch (Exception e) {
		    // TODO Automatisch generierter Erfassungsblock
		    e.printStackTrace();
		}

	    } else {
		ErrorMessage = "State machine interface: Unexpected Data type received!!";
		ErrorFlag = true;
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
