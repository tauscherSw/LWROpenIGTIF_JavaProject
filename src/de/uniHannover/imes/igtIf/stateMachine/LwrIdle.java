/*=========================================================================

  Program:   LWRIdle
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher. Institute of Mechatronics System, Leibniz Universitaet Hannover. All rights reserved.

	See License.txt for more information
	
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
	CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
	EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
	PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
	NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	=========================================================================*/
package de.uniHannover.imes.igtIf.stateMachine;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel
.CartesianImpedanceControlMode;

/**
 * In this state the LWR is holding its position with a maximum stiffness. This
 * is an save mode to which the robot could always change to.
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */
class LwrIdle implements ILwrState {
    /** Flag for increasing the stiffness params of the robot.*/
    private boolean incStiffness = false;

    /**
     * In this function control mode parameters are set and the command pose are
     * calculated due the current LWR State. During the Idle state the Cartesian
     * Stiffness is set to the maximum value of 5000, the NullSpaceStiffness is
     * set to zero and the Pose is set to the measured pose in initializing
     * routine (InitFlag of state machine) is set true of the state. If the
     * change in the stiffness values is exceeding a certain threshold, the
     * stiffness is increased in smaller steps to avoid high forces resulting
     * from a fast increased stiffness.
     * 
     * @param lwrStatemachine
     *            The operated state machine
     * @see ILwrState
     */
    @Override
    public void calcControlParam(final LwrStatemachine lwrStatemachine) {
	double aTransStiffVal = 5000;
	double aRotStiffVal = 300;
	CartesianImpedanceControlMode cartImp = 
		(CartesianImpedanceControlMode) lwrStatemachine.controlMode;
	int[] deltaStiffness = new int[6];
	int[] newStiffness = { (int) aTransStiffVal, (int) aTransStiffVal,
		(int) aTransStiffVal, (int) aRotStiffVal, (int) aRotStiffVal,
		(int) aRotStiffVal };
	for (int i = 0; i < 6; i++) {
	    deltaStiffness[i] = newStiffness[i]
		    - lwrStatemachine.curCartStiffness[i];
	}
	Vector deltaStiffnessTrans = Vector.of(deltaStiffness[0],
		deltaStiffness[1], deltaStiffness[2]);
	Vector deltaStiffnessRot = Vector.of(deltaStiffness[3],
		deltaStiffness[4], deltaStiffness[5]);

	if (lwrStatemachine.InitFlag) {

	    if (deltaStiffnessTrans.length() <= 100
		    && deltaStiffnessRot.length() <= 30) {
		cartImp.parametrize(CartDOF.TRANSL)
			.setStiffness(aTransStiffVal);
		cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
		cartImp.setNullSpaceStiffness(0.);
		lwrStatemachine.controlMode = cartImp;
		lwrStatemachine.cmdPose = lwrStatemachine.curPose;
		incStiffness = false;
		lwrStatemachine.curCartStiffness = newStiffness;
	    } else {
		incStiffness = true;

	    }
	    lwrStatemachine.InitFlag = false;
	}
	if (incStiffness) {
	    boolean increaseTrans = true;
	    boolean increaseRot = true;
	    if (deltaStiffnessTrans.length() > 100) {
		for (int k = 0; k < 3; k++) {
		    if (lwrStatemachine.curCartStiffness[k] <= 4950) {
			newStiffness[k] = lwrStatemachine.curCartStiffness[k] + 50;
		    } else {
			newStiffness[k] = 5000;
		    }
		    lwrStatemachine.curCartStiffness[k] = newStiffness[k];
		}
		cartImp.parametrize(CartDOF.X).setStiffness(newStiffness[0]);
		cartImp.parametrize(CartDOF.Y).setStiffness(newStiffness[1]);
		cartImp.parametrize(CartDOF.Z).setStiffness(newStiffness[2]);
		increaseTrans = true;
	    } else {
		cartImp.parametrize(CartDOF.TRANSL)
			.setStiffness(aTransStiffVal);
		increaseTrans = false;
	    }
	    if (deltaStiffnessRot.length() > 30) {
		for (int k = 3; k < 6; k++) {
		    if (lwrStatemachine.curCartStiffness[k] <= 300 - 15) {
			newStiffness[k] = lwrStatemachine.curCartStiffness[k] + 15;

		    } else {
			newStiffness[k] = 300;
		    }
		    lwrStatemachine.curCartStiffness[k] = newStiffness[k];
		}
		cartImp.parametrize(CartDOF.A).setStiffness(newStiffness[3]);
		cartImp.parametrize(CartDOF.B).setStiffness(newStiffness[4]);
		cartImp.parametrize(CartDOF.C).setStiffness(newStiffness[5]);
		increaseRot = true;
	    } else {
		cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
		increaseRot = false;
	    }
	    lwrStatemachine.curCartStiffness = newStiffness;
	    cartImp.setNullSpaceStiffness(0.);
	    lwrStatemachine.controlMode = cartImp;
	    if (!increaseRot && !increaseTrans) {
		incStiffness = false;
	    }
	}
    }

    /**
     * In this function the Command String which is received from the State
     * Control is interpreted and the parameters are set. It is just called
     * after a State transition For the LWRState LWRIdle this is empty because
     * no Parameters are send from the state control.
     * 
     * @param lwrStatemachine
     *            - The operated state machine
     * @see ILwrState
     */

    @Override
    public void interpretCmdPacket(final LwrStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub
	if (lwrStatemachine.IGTLdatatype.equals("STRING")) {
	    String cmdString;

	    cmdString = lwrStatemachine.CmdIGTmessage;
	    lwrStatemachine.ParameterString = cmdString.substring(cmdString
		    .indexOf(";"));

	}
    }

    /**
     * In this function the acknowledge string which is send to the State
     * Control is defined due the current LWR State. In the Idle State the
     * String is Set to "IDLE;".
     * 
     * @param lwrStatemachine
     *            The operated Statemachine
     */
    @Override
    public void setAckPacket(final LwrStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub
	String ack;
	ack = "IDLE;";
	// Send the string to StateControl
	if (lwrStatemachine.End) {
	    lwrStatemachine.AckIGTmessage = "SHUTDOWN;";
	} else {
	    lwrStatemachine.AckIGTmessage = ack;
	}
    }

}
