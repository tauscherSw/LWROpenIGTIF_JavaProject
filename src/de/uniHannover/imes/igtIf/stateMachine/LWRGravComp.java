package de.uniHannover.imes.igtIf.stateMachine;

import org.medcare.igtl.messages.StringMessage;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * In This State the LWR is set to GravitationsCompensation mode so that robot
 * can be moved without constraints. *
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */
public class LWRGravComp implements LWRState {

    /**
     * In this Function control Mode Parameters are set and the commanded pose
     * are calculated due the current LWR State. In the GravComp State the
     * Translational and rotational Stiffness is set to zero and the command
     * pose to the current measured Pose. Because the Values are static whiles
     * this State is active the Values are just set when the InitFlag of the
     * state machine is true.
     * 
     * @param lwrStatemachine
     *            - The operated state machine
     * @see LWRState
     */
    @Override
    public void CalcControlParam(LWRStatemachine lwrStatemachine) {

	if (lwrStatemachine.InitFlag == true) {
	    // We are in CartImp Mode,
	    // Modify the settings:
	    // NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
	    // NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
	    // WILL DESTABILIZE THE CONTROLLER
	    CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;
	    cartImp.parametrize(CartDOF.ALL).setStiffness(0.0);
	    cartImp.setNullSpaceStiffness(0.);

	    lwrStatemachine.controlMode = cartImp;
	    lwrStatemachine.InitFlag = false;
	}
	if (lwrStatemachine.cmdPose.getTranslation()
		.subtract(lwrStatemachine.curPose.getTranslation()).length() > 100) {

	    System.out.println("Difference to big!!!!!!!!!!!!");
	}

	lwrStatemachine.cmdPose = lwrStatemachine.curPose;

    }

    /**
     * In this Function the Acknowledge String which is send to the State
     * Control is defined due the current LWR State. In the GravCompState the
     * String is Set to "GravComp;".
     * 
     * @param lwrStatemachine
     *            The operated Statemachine
     */
    @Override
    public void SetACKPacket(LWRStatemachine lwrStatemachine) {

	String ACK;
	String ACKname = "ACK_" + String.valueOf(lwrStatemachine.UID);
	ACK = "GravComp;";
	// Send the string to StateControl
	StringMessage String = new StringMessage(ACKname, ACK);
	lwrStatemachine.AckIGTmessage = String;// TODO Automatisch generierter
					       // Methodenstub

    }

    /**
     * In this Function the Command String which is received from the State
     * Control is interpreted and the parameters are set. It is just called
     * after a State transition For the LWRState LWRGravComp this is empty
     * because no Parameters are send from the state control.
     * 
     * @param lwrStatemachine
     *            - The operated state machine
     * @see LWRState
     */

    @Override
    public void InterpretCMDPacket(LWRStatemachine lwrStatemachine) {

    }

}
