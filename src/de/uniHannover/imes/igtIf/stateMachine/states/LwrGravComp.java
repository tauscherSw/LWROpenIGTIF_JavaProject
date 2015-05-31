/*=========================================================================

  Program:   LwrGravComp
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

package de.uniHannover.imes.igtIf.stateMachine.states;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine;
import de.uniHannover.imes.igtIf.communication.control.CommandPacket;
import de.uniHannover.imes.igtIf.communication.control.RobotDataSet;
import de.uniHannover.imes.igtIf.logging.DummyLogger;

/**
 * In this state the LWR is set to gravitation Compensation mode so that robot
 * can be moved manually and freely without constraints. The GravComp/Free mode
 * can be used for e.g., for registration purposes
 *
 */
public class LwrGravComp implements ILwrState {

    //**************************Constants**********************/
    /**
     * Maximum translation deviation allowed.
     */
    private static final int MAX_TRANSLATION = 100;

    //**************************Components*********************/
    /** The logging object for logging output. */
    private ITaskLogger log = new DummyLogger();


    //***************************Methods***********************/
    /**
     * In this Function control mode parameters are set and the command pose is
     * calculated due to the current LWR State. In the GravComp State the
     * translational and rotational Stiffness is set to zero and the command
     * pose to the current measured Pose. Because the values are not changing
     * during this State the values are just set when the InitFlag of the state
     * machine is true.
     * 
     * @param lwrStatemachine
     *            - The operated state machine
     * @see ILwrState
     */
    @Override
    public final void calcControlParam(final LwrStatemachine lwrStatemachine,
	    final RobotDataSet currentRobotDataSet) {

	if (lwrStatemachine.stateChanged) {
	    // We are in CartImp Mode,
	    // Modify the settings:
	    // NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
	    // NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
	    // WILL DESTABILIZE THE CONTROLLER
	    int[] newStiffnessParam = { 0, 0, 0, 0, 0, 0 };
	    CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;
	    cartImp.parametrize(CartDOF.ALL).setStiffness(0.0);
	    cartImp.setNullSpaceStiffness(0.);
	    lwrStatemachine.curCartStiffness = newStiffnessParam;
	    lwrStatemachine.controlMode = cartImp;
	    lwrStatemachine.stateChanged = false;
	}
	if (lwrStatemachine.cmdPose.getTranslation()
		.subtract(currentRobotDataSet.getCurPose().getTranslation())
		.length() > MAX_TRANSLATION) {

	    log.warn("Difference between command and current cartesian"
	    	+ " tcp position is greater than "
		    + MAX_TRANSLATION + "mm.");
	}

	lwrStatemachine.cmdPose = currentRobotDataSet.getCurPose();

    }

    /**
     * In this function the command string which is received from the state
     * control is interpreted and the parameters are set. It is only called
     * after a State transition. For the LWRState LWRGravComp because no
     * Parameters are send from the state control.
     * 
     * @param lwrStatemachine
     *            - The operated state machine
     * @param cmdPacket
     *            the packet to be interpreted.
     * @see ILwrState
     */

    @Override
    public final void interpretCmdPacket(final LwrStatemachine lwrStatemachine,
	    final CommandPacket cmdPacket) {
	if (!cmdPacket.isTransformReceived()) {
	    lwrStatemachine.setParamString(cmdPacket.getCmdString().substring(
		    cmdPacket.getCmdString().indexOf(";")));
	}
    }

    /**
     * In this function the acknowledge string, which is send to the state
     * control is defined due the current LWR State. In the GravComp State the
     * String is Set to "GravComp;".
     * 
     * @param lwrStatemachine
     *            The operated state machine
     */
    @Override
    public final void setAckPacket(final LwrStatemachine lwrStatemachine) {

	String ack;
	ack = "GravComp;";
	lwrStatemachine.setAckIgtMsg(ack);

    }

    @Override
    public final void setLogger(final ITaskLogger extlogger) {
	if (null == extlogger) {
	    throw new NullPointerException("External logger is null");
	}
	log = extlogger;

    }
}
