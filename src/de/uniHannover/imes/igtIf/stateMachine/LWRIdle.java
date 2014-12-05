package de.uniHannover.imes.igtIf.stateMachine;

import org.medcare.igtl.messages.StringMessage;

import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * In This State the LWR is holding its position with a maximum stiffness. This
 * an save mode the robot could always change too.
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */
class LWRIdle implements LWRState {
    /**
     * In this Function control Mode Parameters are set and the commanded pose
     * are calculated due the current LWR State. During the Idle State the
     * Cartesian Stiffness is set to the maximum value of 5000, the
     * NullSpaceStiffness is set to zero and the Pose is set to the measured
     * pose. Because the Values are static whiles this State is active the
     * Values are just set when the InitFlag of the Statemachine is true.
     * 
     * @param lwrStatemachine
     *            The operated state machine
     * @see LWRState
     */
    @Override
    public void CalcControlParam(LWRStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub
	double aTransStiffVal = 5000;
	double aRotStiffVal = 300;
	if (lwrStatemachine.InitFlag == true) {
	    CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;

	    cartImp.setStiffness(aTransStiffVal, aTransStiffVal,
		    aTransStiffVal, aRotStiffVal, aRotStiffVal, aRotStiffVal);
	    cartImp.setNullSpaceStiffness(0.);
	    lwrStatemachine.controlMode = cartImp;
	    lwrStatemachine.cmdPose = lwrStatemachine.curPose;
	    lwrStatemachine.InitFlag = false;
	}
    }

    /**
     * In this Function the Acknowledge String which is send to the State
     * Control is defined due the current LWR State. In the Idle State the
     * String is Set to "IDLE;" or "SendData;".
     * 
     * @param lwrStatemachine
     *            The operated Statemachine
     */
    @Override
    public void SetACKPacket(LWRStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub
	String ACK;
	ACK = "IDLE;";
	String ACKname = "ACK_" + String.valueOf(lwrStatemachine.UID);
	// Send the string to StateControl
	StringMessage String = new StringMessage(ACKname, ACK);
	lwrStatemachine.AckIGTmessage = String;// TODO Automatisch generierter
					       // Methodenstub
    }

    /**
     * In this Function the Command String which is received from the State
     * Control is interpreted and the parameters are set. It is just called
     * after a State transition For the LWRState LWRIdle this is empty because
     * no Parameters are send from the state control.
     * 
     * @param lwrStatemachine
     *            - The operated state machine
     * @see LWRState
     */

    @Override
    public void InterpretCMDPacket(LWRStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub

    }

}
