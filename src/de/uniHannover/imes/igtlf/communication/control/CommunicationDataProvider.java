package de.uniHannover.imes.igtlf.communication.control;

import java.nio.ByteBuffer;

import openIGTLink.swig.IGTLheader;
import openIGTLink.swig.IGTLstring;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

import de.uniHannover.imes.igtlf.communication.OpenIGTLMessage;
import de.uniHannover.imes.igtlf.communication.messages.Command;
import de.uniHannover.imes.igtlf.communication.messages.UnknownCommandException;

/**
 * This class represents the communication data, which is exchanged via the
 * OpenIGTLink protocol. It consists basically of command-object (sent by
 * openIGTL Client) and an ack-object (sent by openIGTL server as response).
 *
 */
public class CommunicationDataProvider {

    /**
     * The string which represents, that a message has to be interpreted as a
     * command.
     */
    private static final String MESSAGE_TYPE_COMMAND = "STRING";
    /**
     * The string which represents, that a message has to be interpreted as a
     * transformation to an external base.
     */
    private static final String MESSAGE_TYPE_TRANSFORM = "TRANSFORM";

    /** Number of elements of a rotational matrix. */
    private static final int SIZE_OF_ROTATION = 9;

    /** Number of elements of a translation vector. */
    private static final int SIZE_OF_TRANS = 3;
    /**
     * Maximum allowed number of repetitive received equal uids.
     */
    private static final int MAX_EQUAL_UIDS = 4;

    // private List<Command> listOfCommands = new ArrayList<Command>();

    /** Logging object. */
    private ITaskLogger logger;

    /** the current and newest command received via openIGTL. */
    private Command currentCommand = null;
    /**
     * The current and newest external transformation matrix received via
     * openIGTL.
     */
    private MatrixTransformation currentExternalTrafo = null;

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
     * 
     * @param message
     *            the message received from another OpenIGTL client as a
     *            command.
     * @return false if "new message" has not been processed, cause it either
     *         hasn't had any data or had no new data.
     */
    public final boolean setNewCmdMessage(final OpenIGTLMessage message) {

	/*
	 * Preliminary checks of the arguments.
	 */
	// Check arguments
	// if(!hasAnyData(final byte[] headerByte, final byte[] bodyByte))
	// {
	// return false;
	// }
	//
	// If the current processed message is the first one, fields have to be
	// initialized.
	if (currentCommand == null) {
	    currentCommand = new Command(0, "IDLE;");
	}

	/*
	 * Process header bytes and extract information.
	 */
	final String messageType = getMessageType(message.getHeader());
	final String deviceName = getDeviceName(message.getHeader());
	long uid = 0;
	// Read uid only if message type was a command.
	if (messageType.equalsIgnoreCase(MESSAGE_TYPE_COMMAND)) {
	    uid = getUid(deviceName);
	    updateUIDStatistics(uid);
	}

	/*
	 * Check if received message contains new data. When message type is a
	 * transform or the received uid is newer than the old go on, otherwise
	 * return.
	 */
	if (messageType.equalsIgnoreCase(MESSAGE_TYPE_TRANSFORM)
		|| currentCommand.getUid() < uid) {

	    /*
	     * Process body bytes and extract information.
	     */
	    if (messageType.equalsIgnoreCase(MESSAGE_TYPE_COMMAND)) {
		final String cmdString = getCommandString(message.getBody());
		currentCommand = new Command(uid, cmdString);

	    } else if (messageType.equalsIgnoreCase(MESSAGE_TYPE_TRANSFORM)) {
		currentExternalTrafo = getTrafo(message.getBody());

	    } else {
		throw new UnknownCommandException("Message type: "
			+ messageType + " is unknown.");
	    }

	    return true;
	} else {
	    return false;
	}

    }

    /**
     * Evaluates the unified id of the processed messages, to find repetitive
     * sent commands.
     * 
     * @param uid
     *            the uid of the current message.
     */
    private void updateUIDStatistics(final long uid) {
	uidDelay = uid - currentCommand.getUid();
	if (uidDelay == 0) {
	    if (uidRepeat == 0) {
		uidRepeatCount++;
	    }
	    uidRepeat++;
	    if (uidRepeatMax < uidRepeat) {
		uidRepeatMax = uidRepeat;
	    }
	    if (uidRepeat >= MAX_EQUAL_UIDS) {
		logger.error("State machine interface: UID has not changed for the "
			+ uidRepeat + ". time!! Check state control!");
	    }
	} else if (uidDelay > 1) {
	    uidMiss = uidMiss + uidDelay - 1;
	    logger.error("State machine interface: missed UID!!(miss count: "
		    + uidMiss + ")");

	} else if (uidDelay == 1) {
	    uidRepeat = 0;
	}
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
		- IGTLstring.IGTL_STRING_HEADER_SIZE];
	int p = 0;
	final int readBeginPos = IGTLstring.IGTL_STRING_HEADER_SIZE;
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
	byte[] tmpDeviceName = new byte[IGTLheader.IGTL_HEADER_DEVSIZE];
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
     * Thread safe getter for the current command received via openIGTL.
     * 
     * @return the current command object.
     */
    public final synchronized Command getCurrentCommand() {
	return currentCommand;
    }

    /**
     * Thread safe getter for the current external transformation received via
     * openIGTL.
     * 
     * @return the transformation object.
     */
    public final synchronized MatrixTransformation getCurrentExtTransform() {
	return currentExternalTrafo;
    }

}