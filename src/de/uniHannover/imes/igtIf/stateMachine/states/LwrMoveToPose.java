/*=========================================================================

  Program:   LWRMoveToPose
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

package de.uniHannover.imes.igtIf.stateMachine.states;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine;
import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine.OpenIGTLinkErrorCode;
import de.uniHannover.imes.igtlf.communication.control.CommandPacket;
import de.uniHannover.imes.igtlf.communication.control.CommunicationDataProvider;
import de.uniHannover.imes.igtlf.communication.control.RobotDataSet;
import de.uniHannover.imes.igtlf.logging.DummyLogger;

/**
 * In This State the LWR is moving to a specified position in Cartesian space.
 * Therefore, the stiffness is set to the maximum value.
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */
public class LwrMoveToPose implements ILwrState {
    /** Cartesian-rotational stiffness in Nm/rad. */
    private static final int CART_ROT_STIFFNESS = 300;
    /** Cartesian-translational stiffness in N/m. */
    private static final int CART_TRANSL_STIFFNESS = 5000;
    /** Distance in mm, when path is interpreted as end-reached. */
    private static final double END_OF_PATH_DEVIATION = 10;
    /** Step length of movement in a cycle in mm. */
    private static final double LENGTH_OF_SINGLE_STEP = 10;
    
    /** The logging object for logging output.*/
    private ITaskLogger log = new DummyLogger();

    /** Current point on the linear path which is closest to the robot position */
    private Vector currentPointLine = null;
    /** start point of the linear path towards target position */
    private Vector startPointLine = null;
    /**
     * Flag indicating if the robot has reached the end of the commanded path.
     */
    private boolean endOfPathFlag = false;
    /** Flag indicating if the robots pose data is represented in imagespace. */
    private boolean imageSpace = false;
    /**
     * Lambda which fulfills the equation: x_end = u*lambdaEnd +
     * currentPointLine .
     */
    private double lambdaEnd = 0.0;
    /**
     * Lambda which fulfills the equation: x_end = u*lambdaNull + startPointLine
     * .
     */
    private double lambdaNull = 0.0;

    /** Orientation of the target frame. */
    private MatrixTransformation targetOri = null;
    /** Translation of the target frame. */
    private Vector targetPos = null;

    /**
     * Norm vector containing the direction of the linear path from strat point
     * to End point.
     */
    private Vector directionLine = null;

    /**
     * In this function control mode parameters are set and the command pose are
     * calculated based on the current LWR State. During the MoveToPose State
     * the Cartesian Stiffness is set to the maximum value of 5000 and the Pose
     * is set to the closest point at the path plus an offset in the desired
     * direction. When the End point of the path is reached the robot holds his
     * position and the boolean EndPath is set true.
     * 
     * @param lwrStatemachine
     *            The operated state machine
     * @see LWRState
     */
    public final void calcControlParam(final LwrStatemachine lwrStatemachine,
	    final RobotDataSet currentRobotDataSet) {

	Vector curPosition = null;
	Vector aim = null;
	double d = 0.0;
	double lambda = 0.0;

	CartesianImpedanceControlMode cartImp = 
		(CartesianImpedanceControlMode) lwrStatemachine.controlMode;
	    MatrixTransformation currentPose = currentRobotDataSet.getCurPose();

	if (lwrStatemachine.stateChanged) {
	    cartImp.parametrize(CartDOF.TRANSL).setStiffness(
		    CART_TRANSL_STIFFNESS);
	    cartImp.parametrize(CartDOF.ROT).setStiffness(CART_ROT_STIFFNESS);
	    cartImp.setNullSpaceStiffness(0);
	    int[] newStiffness = { CART_TRANSL_STIFFNESS,
		    CART_TRANSL_STIFFNESS, CART_TRANSL_STIFFNESS,
		    CART_ROT_STIFFNESS, CART_ROT_STIFFNESS, CART_ROT_STIFFNESS };
	    lwrStatemachine.curCartStiffness = newStiffness;

	    currentPointLine = Vector.of(currentPose
		    .getTranslation().getX(), currentPose
		    .getTranslation().getY(), currentPose
		    .getTranslation().getZ());
	    startPointLine = currentPointLine;
	    directionLine = targetPos.subtract(currentPointLine).normalize();
	    lambdaEnd = targetPos.subtract(currentPointLine).length();
	    lwrStatemachine.stateChanged = false;
	}
	curPosition = currentPose.getTranslation();
	if (!endOfPathFlag) {
	    if (curPosition.subtract(targetPos).length() < END_OF_PATH_DEVIATION) {
		lwrStatemachine.cmdPose = MatrixTransformation.of(targetPos,
			targetOri.getRotationMatrix());
		endOfPathFlag = true;
	    } else {

		d = curPosition.dotProduct(directionLine);
		lambda = d - directionLine.dotProduct(currentPointLine);
		lambdaNull = d - directionLine.dotProduct(startPointLine);
		if (lambdaNull >= 0 && lambdaNull <= lambdaEnd) {
		    currentPointLine = currentPointLine.add(directionLine
			    .multiply(lambda));

		}
		aim = directionLine.multiply(LENGTH_OF_SINGLE_STEP);

		lwrStatemachine.cmdPose = MatrixTransformation.of(
			currentPointLine.add(aim),
			targetOri.getRotationMatrix());
	    }
	} else {
	    lwrStatemachine.cmdPose = MatrixTransformation.of(targetPos,
		    targetOri.getRotationMatrix());
	}

	// Send the new Stiffness settings down to the
	// controller
	lwrStatemachine.controlMode = cartImp;

    }

    /**
     * In this Function the Command String which is received from the State
     * Control is read and interpreted based on to the Current State and if
     * requested and allowed the State is Changed. In The MoveToPose State the
     * allowed state transitions are:<br>
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
     * @param cmdPacket
     *            the packet to be interpreted.
     * @see ILwrState
     */
    @Override
    public final void interpretCmdPacket(final LwrStatemachine lwrStatemachine,
	    final CommandPacket cmdPacket) {
	if (!cmdPacket.isTransformReceived()) {
	    String cmdString;
	    cmdString = cmdPacket.getCmdString();
	    lwrStatemachine.setParamString(cmdString.substring(cmdString
		    .indexOf(";")));
	    String[] cmdArray = cmdString.split(";");
	    if (cmdArray[1].contentEquals("img")) {
		this.imageSpace = true;
	    } else if (cmdArray[1].contentEquals("rob")) {
		this.imageSpace = false;
	    } else {
		lwrStatemachine.ErrorMessage = ("Unexpected coordinate system "
			+ "(supported are img or plane)");
		lwrStatemachine.ErrorCode = OpenIGTLinkErrorCode.ConfigurationError;
		lwrStatemachine.ErrorFlag = true;
	    }
	    this.targetPos = Vector.of(Double.parseDouble(cmdArray[2]),
		    Double.parseDouble(cmdArray[3]),
		    Double.parseDouble(cmdArray[4]));
	    this.targetOri = MatrixTransformation.of(Vector.of(0, 0, 0),
		    Rotation.ofRad(
			    Math.toRadians(Double.parseDouble(cmdArray[5])),
			    Math.toRadians(Double.parseDouble(cmdArray[6])),
			    Math.toRadians(Double.parseDouble(cmdArray[7]))));
	    if (this.imageSpace && cmdPacket.isTransformReceived()) {
		MatrixTransformation tmp = MatrixTransformation.of(targetPos,
			targetOri.getRotationMatrix());
		tmp = cmdPacket.getTrafo().invert().compose(tmp);
		this.targetPos = tmp.getTranslation();
		this.targetOri = tmp.withTranslation(Vector.of(0, 0, 0));

	    }

	} else {
	    lwrStatemachine.ErrorCode = OpenIGTLinkErrorCode.IllegalInstruction;
	    lwrStatemachine.ErrorMessage = "Unexpected Messagetype recieved! Expected STRING";
	}

    }

    /**
     * In this Function the Acknowledge String which is send to the State
     * Control is defined due the current LWR State. In the MovetoPose State the
     * String is Set to "MovetoPose;true;" or "MoveToPose;false;" according to
     * the value of the boolean ReachedPose.
     * 
     * @param lwrStatemachine
     *            The operated Statemachine
     */
    @Override
    public final void setAckPacket(final LwrStatemachine lwrStatemachine) {

	String ack;
	if (endOfPathFlag) {
	    ack = "MoveToPose;true;";
	} else {
	    ack = "MoveToPose;false;";
	}
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
