/*=========================================================================

  Program:   LwrPathImp
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

import java.util.logging.Logger;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine;
import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine.OpenIGTLinkErrorCode;
import de.uniHannover.imes.igtIf.util.MathUtil;
import de.uniHannover.imes.igtIf.communication.control.CommandPacket;
import de.uniHannover.imes.igtIf.communication.control.RobotDataSet;
import de.uniHannover.imes.igtIf.logging.LwrIgtlLogConfigurator;

/**
 * In This State the LWR can be moved along a linear Path in one direction to a
 * specified position in Cartesian space. Therefore, the stiffness is set
 * according to the distance towards the defined path. Therefor, the current
 * pose of the robot at the beginning of this state (statemachine InitFlag
 * ==true) is used as the start pose.
 * 
 */
public class LwrPathImp implements ILwrState {

    //**************************Constants**********************/
    /** Cartesian rotational stiffness value in Nm/rad. */
    private static final int CART_ROT_STIFFNESS = 300;

    /** Initial cartesian translational stiffness value in N/m. */
     private static final int CART_TRANSL_STIFFNESS_INITIAL = 3000;

    /** Maximum cartesian translational stiffness value in N/m. */
     private static final int CART_TRANSL_STIFFNESS_MAX = 5000;

     /**
      * Allowed euclidean distance from current robot position to the path from
      * start to end pose in mm.
      */
     private static final double TOLERANCE = 10.0;

     
     //**************************Components*********************/
     /**
      * Logging mechanism provided by jdk. In case if debug flag is active, all
      * logging output will be directed to a logfile. Otherwise logging output
      * will be displayed on the smartpad.
      */
     private Logger logger = Logger
 	    .getLogger(LwrIgtlLogConfigurator.LOGGERS_NAME);

    /**
     * vector to define the line path from start to End point. g: x= ap +
     * lambda*u.
     */
    private Vector ap = null;

    /** Distance in mm from the current robot position to the line path. */
    private double distance = 0.0;

    /**
     * Flag indicates if the target pose is reached.
     */
    private boolean endOfPath = false;
    /**
     * Flag indicates if the Position is send in Image or robot base coordinate
     * system.
     */
    private boolean imageSpace = false;
    /**
     * directional vector of the line g: x= ap + lambda*u.
     */
    private double lambdaEnd = 0;

    /**
     * Target Orientation as rotation matrix in Image or robot base coordinate
     * system due to ImageSpace flag.
     */
    private MatrixTransformation targetOrientation = null;
    /**
     * TargetPosition in Image or robot base coordinate system due to ImageSpace
     * flag.
     */
    private Vector targetPosition = null;
     /**
     * Directional vector of the line g: x= ap + lambda*u.
     */
    private Vector u = null;

    
    
    //***************************Methods***********************/
    /**
     * In this Function control Mode Parameters are set and the commanded pose
     * are calculated due the current LWR State. During the PathImp State the
     * Cartesian Stiffness is set according to the distance to defined Path, the
     * NullSpaceStiffness is set to zero and the Pose is set to the closest
     * point at the path plus an offset in the desired direction. When the End
     * point of the path is reached the robot holds his position and the boolean
     * EndPath is set true.
     * 
     * @param lwrStatemachine
     *            The operated state machine
     * @see LWRState
     */
    @Override
    public final void calcControlParam(final LwrStatemachine lwrStatemachine,
	    final RobotDataSet currentRobotDataSet) {

	double d = 0.0;
	distance = 0.0;
	Vector curPosition = null;
	double lambda = 0.0;
	Vector aim = null;
	double stiffVal = CART_TRANSL_STIFFNESS_INITIAL;
	Vector aTransStiffVal = null;
	int[] newStiffness = { 0, 0, 0, 0, 0, 0 };

	MatrixTransformation curPose = currentRobotDataSet.getCurPose();
	
	if (lwrStatemachine.stateChanged) {
	    aim = Vector.of(0, 0, 0);
	    targetOrientation = MatrixTransformation.of(Vector.of(0, 0, 0),
		    curPose.getRotation());
	    ap = Vector.of(curPose.getTranslation().getX(),
		    curPose.getTranslation().getY(),
		    curPose.getTranslation().getZ());
	    lambdaEnd = targetPosition.subtract(ap).length();
	    u = targetPosition.subtract(ap).normalize();
	    lwrStatemachine.stateChanged = false;
	}
	curPosition = curPose.getTranslation();


	if (curPosition.subtract(targetPosition).length() < TOLERANCE) {
		stiffVal = CART_TRANSL_STIFFNESS_MAX;
		aTransStiffVal = Vector.of(stiffVal, stiffVal, stiffVal);
		lwrStatemachine.cmdPose = MatrixTransformation
			.of(targetOrientation.withTranslation(targetPosition));
		if (!endOfPath) {
		    logger.info("Path Impedance: Reached End of path ("
			    + targetPosition + ")");
		}
		endOfPath = true;
	
	} else {

	    d = curPosition.dotProduct(u);
	    lambda = d - u.dotProduct(ap);

	    if (lambda >= 0 && lambda <= lambdaEnd) {
		ap = ap.add(u.multiply(lambda));
	    }
	    distance = ap.subtract(curPosition).length();

	    // Beginn der Impedanzregelung fuer Pfad
	    if (distance > TOLERANCE) { // Bahngrenze uebertreten

		aTransStiffVal = Vector.of(CART_TRANSL_STIFFNESS_MAX,
			CART_TRANSL_STIFFNESS_MAX,CART_TRANSL_STIFFNESS_MAX);
		aim = ap.subtract(curPosition).normalize()
			.multiply(distance - TOLERANCE);
	    } else {
		stiffVal = getStiffness(distance, TOLERANCE);
		aTransStiffVal = Vector.of(stiffVal, stiffVal, stiffVal);
		aim = Vector.of(0, 0, 0);
	    }

	    lwrStatemachine.cmdPose = MatrixTransformation.of(ap.add(aim),
		    targetOrientation.getRotation());
	}
	CartesianImpedanceControlMode cartImp = 
		(CartesianImpedanceControlMode) lwrStatemachine.controlMode;

	if (endOfPath) {
	    lwrStatemachine.cmdPose = MatrixTransformation.of(targetPosition,
		    targetOrientation.getRotation());
	    cartImp.parametrize(CartDOF.TRANSL).setStiffness(CART_TRANSL_STIFFNESS_MAX);
	    cartImp.parametrize(CartDOF.ROT).setStiffness(CART_ROT_STIFFNESS);
	    newStiffness[0] = CART_TRANSL_STIFFNESS_MAX;
	    newStiffness[1] = CART_TRANSL_STIFFNESS_MAX;
	    newStiffness[2] = CART_TRANSL_STIFFNESS_MAX;
	    newStiffness[3] = CART_ROT_STIFFNESS;
	    newStiffness[4] = CART_ROT_STIFFNESS;
	    newStiffness[5] = CART_ROT_STIFFNESS;
	} else {
	    cartImp.parametrize(CartDOF.X).setStiffness(aTransStiffVal.getX());
	    cartImp.parametrize(CartDOF.Y).setStiffness(aTransStiffVal.getY());
	    cartImp.parametrize(CartDOF.Z).setStiffness(aTransStiffVal.getZ());
	    cartImp.parametrize(CartDOF.ROT).setStiffness(CART_ROT_STIFFNESS);
	    cartImp.setNullSpaceStiffness(0.0);
	    newStiffness[0] = MathUtil.longToInt(Math.round(aTransStiffVal.getX()));
	    newStiffness[1] = MathUtil.longToInt(Math.round(aTransStiffVal.getY()));
	    newStiffness[2] = MathUtil.longToInt(Math.round(aTransStiffVal.getZ()));
	    newStiffness[3] = CART_ROT_STIFFNESS;
	    newStiffness[4] = CART_ROT_STIFFNESS;
	    newStiffness[5] = CART_ROT_STIFFNESS;

	}
	// Send the new Stiffness settings down to the
	// controller
	lwrStatemachine.curCartStiffness = newStiffness;
	lwrStatemachine.controlMode = cartImp;
    }

    /**
     * In this Function the Stiffness value is calculated.
     * 
     * @param dist
     *            the minimum distance to path
     * @param tol
     *            the maximum allowed distance from the path
     * @return the stiffness value.
     */

    public final double getStiffness(final double dist, final double tol) {
	/*
	 * Berechnung der Daempfung bei Annaeherung an die Grenze.
	 */
	double maxStiff = CART_TRANSL_STIFFNESS_MAX;
	if (dist >= tol) {
	    return maxStiff;
	} else if (dist < tol && dist > 0) {
	    double y;
	    double m = maxStiff / Math.pow(tol, 2);
	    y = m * Math.pow(dist, 2);
	    return y;
	} else {
	    return 0;
	}
    }

    /**
     * In this Function the CommandIGTMessage which is received from the State
     * Control is read and interpreted due to the Current State and if requested
     * and allowed the State is Changed. In The VirtualFixtures State the
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

	boolean err = false;
	if (!cmdPacket.isTransformReceived()) {
	    String cmdString;
	    cmdString = cmdPacket.getCmdString();
	    lwrStatemachine.setParamString(cmdString.substring(cmdString
		    .indexOf(";")));
	    String[] cmdArray = cmdString.split(";");
	    if (cmdArray[1].contentEquals("img")) {
		this.imageSpace = true;
		logger.info("Image space is set to true...");
	    } else if (cmdArray[1].contentEquals("rob")) {
		this.imageSpace = false;
	    } else {
		err = true;
	    }

	    this.targetPosition = Vector.of(Double.parseDouble(cmdArray[2]),
		    Double.parseDouble(cmdArray[3]),
		    Double.parseDouble(cmdArray[4]));
	    logger.info("Targetposition...: " + this.targetPosition);

	    if (this.imageSpace && cmdPacket.isTransformReceived()) {
		Vector tmp;
		tmp = cmdPacket.getTrafo()
			.invert()
			.applyTo(targetPosition)
			.add(cmdPacket.getTrafo().invert()
				.getTranslation());
		this.targetPosition = tmp;
		logger.info("After Transformation...: "
			+ this.targetPosition);

	    }
	    if (err) {
		lwrStatemachine.ErrorCode = 
			OpenIGTLinkErrorCode.ConfigurationError;
		lwrStatemachine.setNewState(new LwrIdle());
	    }

	} else {
	    lwrStatemachine.ErrorCode = 
		    OpenIGTLinkErrorCode.IllegalInstruction;
	    lwrStatemachine.ErrorMessage = 
		    "Unexpected Messagetype recieved! Expected STRING";
	}
    }

    /**
     * In this Function the Acknowledge IGTMessage which is send to the State
     * Control is defined due the current LWR State. In the PathImp State the
     * IGTMessageString is Set to "PathImp;true;" or "PathImp;false,"according
     * to the value of the boolean PathImp.
     * 
     * @param lwrStatemachine
     *            The operated Statemachine
     */
    @Override
    public final void setAckPacket(final LwrStatemachine lwrStatemachine) {

	String ack;
	if (endOfPath) {
	    ack = "PathImp;0;true;";
	} else {
	    if (distance > TOLERANCE) {
		ack = "PathImp;2;false;";
	    } else if (distance == 0) {
		ack = "PathImp;0;false;";
	    } else {
		ack = "PathImp;1;false;";
	    }
	}

	lwrStatemachine.setAckIgtMsg(ack);
    }
    

}
