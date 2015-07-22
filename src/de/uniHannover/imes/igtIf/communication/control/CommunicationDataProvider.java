/*=========================================================================

  Program:   CommunicationDataProvider
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

import java.nio.ByteBuffer;
import java.util.logging.Logger;

import OpenIGTLink.swig.IGTLheader;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

import de.uniHannover.imes.igtIf.communication.IIGTLMsgInterface;
import de.uniHannover.imes.igtIf.logging.LwrIgtlLogConfigurator;

/**
 * This class represents the communication data, which is exchanged via the
 * OpenIGTLink protocol. It consists basically of command-object (sent by
 * openIGTL Client) and an ack-object (sent by openIGTL server as response).
 * 
 */
public class CommunicationDataProvider {

    /**
     * Represents the different types of igtl-messages.
     */
    public enum IgtlMsgType {

	/** A message containing just a command string. */
	Command("STRING"),
	/**
	 * A message containing an transformation matrix to an external
	 * reference frame.
	 */
	Transform("TRANSFORM");

	/**
	 * The name of the message type in upper letter case for comparability
	 * of incoming messages.
	 */
	private final String curName;

	/**
	 * Constructs a message type.
	 * 
	 * @param name
	 *            the name of the message.
	 */
	private IgtlMsgType(final String name) {
	    curName = name;
	}

	/**
	 * Returns the correct name of the message type.
	 * 
	 * @return the correct name of the message type.
	 */
	public String getTypeName() {
	    return curName;
	}

    }

    /** Number of elements of a rotational matrix. */
    private static final int SIZE_OF_ROTATION = 9;

    /** Number of elements of a translation vector. */
    private static final int SIZE_OF_TRANS = 3;
    /**
     * Maximum allowed number of repetitive received equal uids.
     */
    private static final int MAX_EQUAL_UIDS = 4;

    /**
     * Logging mechanism provided by jdk. In case if debug flag is active, all
     * logging output will be directed to a logfile. Otherwise logging output
     * will be displayed on the smartpad.
     */
    private Logger logger = Logger
	    .getLogger(LwrIgtlLogConfigurator.LOGGERS_NAME);

    /** the current and newest command received via openIGTL. */
    private CommandPacket curPacket = null;

    /**
     * delay loops between receiving and sending a packet with the same UID
     * (should be 0).
     */
    private long uidDelay = 0;

    /**
     * Number of missed UID numbers in total.
     */
    private long uidMiss = 0;

    /**
     * Number loops getting the same UID number (in a row).
     */
    private int uidRepeat = 0;

    /**
     * Number of maximun repetitions in a row.
     */
    private int uidRepeatMax = 0;

    /**
     * Number of detected repetitions of the unified identification number.
     */
    private int uidRepeatCount = 0;

    /**
     * Robot data sink for reading robot data into a cyclic image of the data.
     */
    private final LBR robotDataSink;

    /**
     * The cyclic image of the robot data.
     */
    private RobotDataSet curRobotDataSet;

    /**
     * UID of the Pose gotten via SmartServo. The value is increased each time
     * new data is read from the robot.
     */
    private long poseUid;

    /**
     * Constructs an communication provider. Needs a robot-data-sink for reading
     * data like joint position and so on.
     * 
     * @param dataSink
     *            the robot object, which gains access to cyclic robot data.
     */
    public CommunicationDataProvider(final LBR dataSink) {
	robotDataSink = dataSink;
	poseUid = 0;

    }

    /**
     * Reads a new message, which was previously received via openIGTL.
     * 
     * @param message
     *            the message received from another OpenIGTL client as a
     *            command.
     */
    public final void readNewCmdMessage(final IIGTLMsgInterface message) {
	logger.entering(this.getClass().getName(), "readNewCmdMessage(...)");

	if (null == message) {
	    throw new NullPointerException("Message argument is null");
	}

	/*
	 * Process header bytes and extract information.
	 */
	final String messageType = getMessageType(message.getHeader());
	final String deviceName = getDeviceName(message.getHeader());

	/*
	 * Define the data belonging to the current command packet.
	 */
	IgtlMsgType tmpMsgType = null;
	boolean tmpTransformReceived = true;
	long tmpUid = 0;
	String tmpCmdString;
	MatrixTransformation tmpExtTrafo;

	/*
	 * Preprocessing of message. This means extracting the message type and
	 * checking the uid. If the message's UID is old or already known, then
	 * skip the message. Only messages of type cmd contain a UID!
	 */
	if (messageType.equalsIgnoreCase(IgtlMsgType.Command.getTypeName())) {
	    tmpMsgType = IgtlMsgType.Command;
	    tmpUid = getUid(deviceName); // device-name contains uid.
	    updateUIDStatistics(tmpUid);
	    tmpTransformReceived = false;
	    // if current's msg uid is already known skip whole msg.
	    if (getCurrentCmdPacket().getUid() == tmpUid) {
		logger.fine("Skipping current received msg, "
			+ "because uid is already known.");
		logger.exiting(this.getClass().getName(),
			"readNewCmdMessage(...)");
		if (!curPacket.getCmdString().equalsIgnoreCase(
			getCommandString(message.getBody()))) {
//		    throw new IllegalStateException("Current system's uid: "
//			    + getCurrentCmdPacket().getUid()
//			    + " and received message's uid: " + tmpUid
//			    + " are equal but the command-strings aren't: "
//			    + curPacket.getCmdString() + " vs. "
//			    + getCommandString(message.getBody()));

		}
		return;
	    }
	} else { // otherwise the current msg must hold a transform
	    tmpTransformReceived = true;
	    tmpMsgType = IgtlMsgType.Transform;
	}

	/*
	 * Check if received message contains new data. When message type is a
	 * transform or the received uid is newer than the old go on, otherwise
	 * return.
	 */

	if (getCurrentCmdPacket().getUid() < tmpUid || tmpTransformReceived) {
	    logger.fine("Processing new message...");
	    /*
	     * Process body bytes and extract information.
	     */
	    switch (tmpMsgType) {
	    case Command:
		tmpCmdString = getCommandString(message.getBody());
		logger.fine("New command-message: CmdString: " + tmpCmdString
			+ ", UID: " + tmpUid);
		// use old trafo for filling up field.
		tmpExtTrafo = getCurrentCmdPacket().getTrafo();
		break;
	    case Transform:
		// Use old cmd string to fill up field.
		tmpCmdString = getCurrentCmdPacket().getCmdString();
		// Use old uid to fill up field.
		tmpUid = getCurrentCmdPacket().getUid();
		tmpExtTrafo = getTrafo(message.getBody());
		tmpTransformReceived = true;
		logger.fine("New transform-message: ExtTrafo: "
			+ tmpExtTrafo.toString());
		break;
	    default:
		throw new UnknownCommandException("Message type: "
			+ messageType + " is unknown.");

	    }
	    // Set here new command packet.
	    curPacket = new CommandPacket(tmpCmdString, tmpMsgType, tmpUid,
		    tmpExtTrafo, tmpTransformReceived);
	    logger.finest("New message saved.");

	} else {
	    logger.exiting(this.getClass().getName(), "readNewCmdMessage(...)");
	    throw new IllegalStateException(
		    "UID failure. UID of the received msg (" + tmpUid
			    + ") is older than " + "the state machine's one ("
			    + curPacket.getUid() + ").");
	}

	logger.exiting(this.getClass().getName(), "readNewCmdMessage(...)");

    }

    /**
     * Reads new robot data in a current robot dataset.
     */
    public final void readNewRobotData() {
	logger.entering(this.getClass().getName(), "readNewRobotData()");
	MatrixTransformation currentTcpPose = MatrixTransformation
		.of(robotDataSink.getCurrentCartesianPosition(
			robotDataSink.getFlange()).transformationFromWorld());
	JointPosition currentJointPosition = robotDataSink
		.getCurrentJointPosition();
	Vector tcpForce = robotDataSink.getExternalForceTorque(
		robotDataSink.getFlange()).getForce();
	// for initialization no synchronized access to curRobotDataSet
	if (null == curRobotDataSet) {
	    curRobotDataSet = new RobotDataSet(currentJointPosition,
		    currentTcpPose, tcpForce);
	    logger.finest("Initialized new robot dataset..."
		    + curRobotDataSet.printRobotDataSet());
	}
	// only synchronized access is allowed
	synchronized (curRobotDataSet) {
	    curRobotDataSet = new RobotDataSet(currentJointPosition,
		    currentTcpPose, tcpForce);
	    logger.finest("Saving new robot dataset..."
		    + curRobotDataSet.printRobotDataSet());
	}

	poseUid++;
	logger.exiting(this.getClass().getName(), "readNewRobotData()");
    }

    /**
     * Evaluates the unified id of the processed messages, to find repetitive
     * sent commands.
     * 
     * @param uid
     *            the uid of the current message.
     */
    private void updateUIDStatistics(final long uid) {
	logger.entering(this.getClass().getName(), "updateUIDStatistics(...)");
	uidDelay = uid - curPacket.getUid();
	if (uidDelay == 0) {
	    if (uidRepeat == 0) {
		uidRepeatCount++;
	    }
	    uidRepeat++;
	    if (uidRepeatMax < uidRepeat) {
		uidRepeatMax = uidRepeat;
	    }
	    if (uidRepeat >= MAX_EQUAL_UIDS) {
		logger.severe("State machine interface: UID has not changed for the "
			+ uidRepeat + ". time!! Check state control!");
	    }
	} else if (uidDelay > 1) {
	    uidMiss = uidMiss + uidDelay - 1;
	    logger.severe("State machine interface: missed UID!!(miss count: "
		    + uidMiss + ")");

	} else if (uidDelay == 1) {
	    uidRepeat = 0;
	}
	logger.exiting(this.getClass().getName(), "updateUIDStatistics(...)");
    }

    /**
     * Reads the actual command string from the body bytes.
     * 
     * @param bodyByte
     *            the bytes of the body of the message, which has to be
     *            evaluated.
     * @return the command received within the message.
     */
    private String getCommandString(final byte[] bodyByte) {

	byte[] tmpString = new byte[bodyByte.length
		- OpenIGTLink.swig.IGTLstring.IGTL_STRING_HEADER_SIZE];
	int p = 0;
	final int readBeginPos = OpenIGTLink.swig.IGTLstring.IGTL_STRING_HEADER_SIZE;
	for (int z = readBeginPos; z < bodyByte.length; z++) {
	    tmpString[p] = bodyByte[z];
	    p++;
	}

	return new String(tmpString);
    }

    /**
     * Reads the unified id within a device name.
     * 
     * @param deviceName
     *            the deviceName of the message
     * @return the unified id.
     */
    private long getUid(final String deviceName) {
	if (deviceName.split("_").length >= 2) {
	    final String uidString = deviceName.split("_")[1];
	    long uid = Long.parseLong(uidString);
	    if (Long.MAX_VALUE == uid) {
		throw new ArithmeticException(
			"Unified Id is close to number overflow for datatype long.");
	    }
	    return uid;
	} else {
	    throw new UnknownCommandException(
		    "Illegal length of the splitted device name ("
			    + deviceName.split("_") + "). Length is "
			    + deviceName.split("_").length
			    + " but it has to be greater or equal than 2.");
	}
    }

    /**
     * Reads the device name within the header of a message.
     * 
     * @param headerBytes
     *            the bytes of the header of the message, which has to be
     *            evaluated.
     * @return the device name.
     */
    private String getDeviceName(final byte[] headerBytes) {
	byte[] tmpDeviceName = new byte[OpenIGTLink.swig.IGTLheader.IGTL_HEADER_DEVSIZE];
	int l = 0;
	int enddata = 0;
	final int readBeginPos = 14;
	for (int j = readBeginPos; j < readBeginPos
		+ IGTLheader.IGTL_HEADER_DEVSIZE
		&& enddata == 0; j++) {
	    tmpDeviceName[l] = headerBytes[j];
	    if (headerBytes[j] == 0) {
		enddata = l;
	    }
	    l++;
	}
	return new String(tmpDeviceName).substring(0, enddata);
    }

    /**
     * Reads the message type within the header of a message.
     * 
     * @param headerBytes
     *            the bytes of the header of the message, which has to be
     *            evaluated.
     * @return the message type.
     */
    private String getMessageType(final byte[] headerBytes) {
	int enddata = 0;
	byte[] tmpName = new byte[IGTLheader.IGTL_HEADER_TYPE_SIZE];
	int k = 0;
	for (int i = 2; i < 2 + IGTLheader.IGTL_HEADER_TYPE_SIZE
		&& enddata == 0; i++) {
	    tmpName[k] = headerBytes[i];
	    if (headerBytes[i] == 0) {
		enddata = k;
	    }
	    k++;
	}

	return new String(tmpName).substring(0, enddata);
    }

    /**
     * Reads the current external transformation matrix from the body bytes.
     * 
     * @param bodyBytes
     *            the bytes of the body of the message, which has to be
     *            evaluated.
     * @return the command received within the message.
     */
    private MatrixTransformation getTrafo(final byte[] bodyBytes) {
	ByteBuffer bodyBuff = ByteBuffer.wrap(bodyBytes);
	double[] rot = new double[SIZE_OF_ROTATION];
	double[] trans = new double[SIZE_OF_TRANS];
	for (int i = 0; i < SIZE_OF_ROTATION + SIZE_OF_TRANS; i++) {
	    if (i < SIZE_OF_TRANS) {
		rot[i] = bodyBuff.getDouble(i * Double.SIZE);
	    } else if (i >= SIZE_OF_TRANS
		    && i < SIZE_OF_ROTATION + SIZE_OF_TRANS) {
		trans[i - SIZE_OF_ROTATION] = bodyBuff.getDouble(i
			* Double.SIZE);
	    }
	}
	return MatrixTransformation.of(Vector.of(trans[0], trans[1], trans[2]),
		Matrix.ofRowFirst(rot[0], rot[1], rot[2], rot[3], rot[4],
			rot[5], rot[6], rot[7], rot[8]));
    }

    /**
     * Gives statistics about uid misses.
     * 
     * @return a String containing the number of missed uids, the number of
     *         repeated uids and the maximum allowed number of repeated uids.
     */
    public final synchronized String getUidStatistics() {
	return new String("UID miss: " + uidMiss + " UIDrepeats: "
		+ uidRepeatCount + "(max: " + uidRepeatMax + ")");
    }

    /**
     * Getter for the current and latest command packet received from the
     * openIGTL client.
     * 
     * @return the command packet.
     */
    public final CommandPacket getCurrentCmdPacket() {

	// For initialization
	if (curPacket == null) {
	    // initialize the current command packet.
	    curPacket = new CommandPacket("IDLE;", IgtlMsgType.Command, 0,
		    null, false);
	}

	synchronized (curPacket) {
	    return curPacket;
	}
    }

    /**
     * Getter for the current poseUid.
     * 
     * @return the poseUid
     */
    public final long getPoseUid() {
	return poseUid;
    }

    /**
     * Getter for the current robot dataset.
     * 
     * @return the current robot dataset.
     */
    public final RobotDataSet getCurRobotDataSet() {
	synchronized (curRobotDataSet) {
	    return curRobotDataSet;
	}
    }

}
