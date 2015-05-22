package de.uniHannover.imes.igtIf.communication.control;

import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;

import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine;
import de.uniHannover.imes.igtIf.communication.control.CommunicationDataProvider.IgtlMsgType;

/**
 * This class represents one single cyclic dataset, which is given to the
 * LWRStateMachine ({@linkplain LwrStatemachine}. It holds data like the current
 * command string, the message type, the uid and eventually an transformation
 * matrix to an external frame. This class is used to summarize all the data and
 * to make sure, that the data is consistent. This is realized by only one
 * setter. Data can be accessed via multiple getters.
 */
public class CommandPacket {

	/** The command string. */
	private final String cmdString;

	private final IgtlMsgType msgType;
	/** The unified ID of the corresponding command. */
	private final long uId;
	private final MatrixTransformation trafo;
	private final boolean transformReceived;

	public CommandPacket(final String currentCmdString,
			final IgtlMsgType currentMsgType, final long currentUid,
			final MatrixTransformation currentExternalTrafo,
			final boolean curTransformReceived) {
		cmdString = currentCmdString;
		msgType = currentMsgType;
		uId = currentUid;
		trafo = currentExternalTrafo;
		transformReceived = curTransformReceived;

	}

	/**
	 * Getter for the unified id of this command object.
	 * 
	 * @return the unified id of this command object.
	 */
	public final long getUid() {
		return uId;
	}

	/**
	 * Getter of the command string of this command object.
	 * 
	 * @return the command string.
	 */
	public final String getCmdString() {
		return cmdString;
	}

	public final IgtlMsgType getMsgType() {
		return msgType;
	}

	public final MatrixTransformation getTrafo() {
		return trafo;
	}

	public final boolean isTransformReceived() {
		return transformReceived;
	}

	public final String printDetails() {
		if (null != trafo) {
			return "MessageType: " + msgType + "\n" + "Command String: "
					+ cmdString + "\n" + "UID: " + uId + "\n" + "Trafo: "
					+ trafo.toString() + "\n" + "Transform received: "
					+ transformReceived + "\n";
		} else {
			return "MessageType: " + msgType + "\n" + "Command String: "
					+ cmdString + "\n" + "UID: " + uId + "\n"
					+ "Transform received: " + transformReceived + "\n";
		}
	}

}
