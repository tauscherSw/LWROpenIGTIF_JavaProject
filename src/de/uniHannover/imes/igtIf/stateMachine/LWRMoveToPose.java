package de.uniHannover.imes.igtIf.stateMachine;

import java.io.UnsupportedEncodingException;

import org.medcare.igtl.messages.StringMessage;

import com.kuka.common.ThreadUtil;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.math.MatrixRotation;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * In This State the LWR is moving to a specified position in Cartesian space.
 * Therefore, the stiffness is set to the maximum value.
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */
public class LWRMoveToPose implements LWRState {
    private boolean EndofPath = false;
    public boolean ImageSpace = false;
    public Vector TargetPosition = null;
    public MatrixTransformation TargetOrientation = null;

    @Override
    /**
     * In this Function control Mode Parameters are set and the commanded pose are calculated due the current LWR State.
     * During the MoveToPose State the Cartesian Stiffness is set to the maximum value of 5000 and the Pose is set to the closest point at the path plus an offset in the desired direction.
     * When the End point of the path is reached the robot holds his position and the boolean EndPath is set true.
     * @param lwrStatemachine The operated state machine
     * @see LWRState
     */
    public void CalcControlParam(LWRStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub

	CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;

	if (lwrStatemachine.InitFlag) {
	    cartImp.parametrize(CartDOF.TRANSL).setStiffness(5000);
	    cartImp.parametrize(CartDOF.ROT).setStiffness(300);
	    cartImp.setNullSpaceStiffness(0.0);
	    System.out.println("Move to Pose: Moving to : " + TargetPosition);
	    // lwrStatemachine.cmdPose = MatrixTransformation.of(TargetPosition,
	    // TargetOrientation.getRotationMatrix());
	    lwrStatemachine.InitFlag = false;
	}
	if (lwrStatemachine.curPose == MatrixTransformation.of(TargetPosition,
		TargetOrientation.getRotationMatrix()) && !EndofPath) {
	    System.out
		    .println("Move to Pose: Reached Target Position and know holding it.");
	    EndofPath = true;
	}
	lwrStatemachine.cmdPose = MatrixTransformation.of(TargetPosition,
		TargetOrientation.getRotationMatrix());
	// Send the new Stiffness settings down to the
	// controller
	lwrStatemachine.controlMode = cartImp;

    }

    /**
     * In this Function the Acknowledge String which is send to the State
     * Control is defined due the current LWR State. In the MovetoPose State the
     * String is Set to "MovetoPose;true;" or "MoveToPose;false;" according to
     * teh value of the boolean ReachedPose.
     * 
     * @param lwrStatemachine
     *            The operated Statemachine
     */
    @Override
    public void SetACKPacket(LWRStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub
	String ACK;
	if (EndofPath) {
	    ACK = "MoveToPose;true;";
	} else {
	    ACK = "MoveToPose;false;";
	}
	String ACKname = "ACK_" + String.valueOf(lwrStatemachine.UID);
	// Send the string to StateControl
	StringMessage String = new StringMessage(ACKname, ACK);
	lwrStatemachine.AckIGTmessage = String;// TODO Automatisch generierter
					       // Methodenstub
    }

    /**
     * In this Function the Command String which is received from the State
     * Control is read and interpreted due to the Current State and if requested
     * and allowed the State is Changed. In The MoveToPose State the allowed
     * state transitions are:<br>
     * - IDLE (transition condition: none)<br>
     * - GravComp (transition condition: none)<br>
     * - VirtualFixtures (transition condition: RegistrationFinished AND
     * DataSend AND TransformRecieved)<br>
     * - MoveToPose (transition condition: RegistrationFinished AND DataSend AND
     * TransformRecieved)<br>
     * If a transition request to another State is requested or some the number
     * of Parameters is incorrect the state is set to Error and the Error code
     * is set to 1 = Transition not allowed.
     * 
     * @param lwrStatemachine
     *            - The operated state machine
     * @see LWRState
     */
    @Override
    public void InterpretCMDPacket(LWRStatemachine lwrStatemachine) {
	boolean Error = false;
	if (lwrStatemachine.CmdIGTmessage.getHeader().getDataType()
		.equals("STRING")) {
	    String CMD_String;
	    try {
		CMD_String = new String(
			lwrStatemachine.CmdIGTmessage.getBody(), 4,
			lwrStatemachine.CmdIGTmessage.getBody().length - 4,
			"US-ASCII");
		String[] CMD_Array = CMD_String.split(";");
		if (CMD_Array[1].contentEquals("img")) {
		    this.ImageSpace = true;
		} else if (CMD_Array[1].contentEquals("rob")) {
		    this.ImageSpace = false;
		} else {
		    Error = true;
		    lwrStatemachine.ErrorMessage = ("Unexpected coordinate system (supported are img or plane)");
		    lwrStatemachine.ErrorCode = 10;
		    lwrStatemachine.ErrorFlag = true;
		}
		this.TargetPosition = Vector.of(
			Double.parseDouble(CMD_Array[2]),
			Double.parseDouble(CMD_Array[3]),
			Double.parseDouble(CMD_Array[4]));
		this.TargetOrientation = MatrixTransformation.of(
			Vector.of(0, 0, 0),
			Rotation.ofRad(Double.parseDouble(CMD_Array[5])
				* Math.PI / 180,
				Double.parseDouble(CMD_Array[6]) * Math.PI
					/ 180, Double.parseDouble(CMD_Array[7])
					* Math.PI / 180));

	    } catch (UnsupportedEncodingException e) {
		// TODO Automatisch generierter Erfassungsblock
		e.printStackTrace();
	    }
	} else {
	    lwrStatemachine.ErrorCode = 12; // Illegal/unknown instruction
	    lwrStatemachine.ErrorMessage = "Unexpected Messagetype recieved! Expected STRING";
	}

    }

}
