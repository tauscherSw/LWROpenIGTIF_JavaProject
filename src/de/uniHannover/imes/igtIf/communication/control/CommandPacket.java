/*=========================================================================

  Program:   CommandPacket
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
