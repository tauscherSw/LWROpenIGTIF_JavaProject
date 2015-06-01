/*=========================================================================

  Program:   LwrVirtualFixtures
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
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine;
import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine.OpenIGTLinkErrorCode;
import de.uniHannover.imes.igtIf.communication.control.CommandPacket;
import de.uniHannover.imes.igtIf.communication.control.RobotDataSet;
import de.uniHannover.imes.igtIf.logging.DummyLogger;

/**
 * In this State two different kinds of Virtuall Fixtures, a plane and a cone
 * are definied and the Stiffness and Damping values are calculated due to there
 * distance to the defined Fixtures. This is one of the States defined for the
 * LWRStatemachine see LWRState. There are three different areas defined: the
 * Awareness area, the free area and the forbidden area. In the first area the
 * stiffness values are increasing when the robot is getting closer to the
 * Fixtures and decreasing if the distance increases. In the second can move
 * freely and in the third area the stiffness values are set to a maximum value
 * and the commanded Position is the closest point on the Fixture.
 * 
 */

public class LwrVirtualFixtures implements ILwrState {

    //**************************Constants**********************/
    /**
     * Enum for the virtual fixtures type {Cone, plane }.
     */
    private static enum VirtualFixtureType {
	/** The virtual fixture has a plane geometry. */
	Plane,
	/** The virtual fixture has a cone geometry. */
	Cone,
    }
    
  //TODO @Sebastian further use?
    private static final int CONE_TOLERANCE_ENDPOINT = 10;

    /**
     * Maximum cartesian-translational stiffness of the lwr in N/m.
     */
    private static final double CART_TRANSL_STIFFNESS_MAX = 5000;

    /** Number of vectors for mean calculation of force vector. */
    private static final int NUM_FORCE_NORM_VECTORS = 5;

    
    //**************************Flags**************************/
    /**
     * Flag indicates if the robot is near the tip of the cone. Due to this
     * variable the half sphere used as virtual fixture instead of the cone tip
     * is used to avoid stability issues.
     */
    private boolean coneTip = false;
    
    /** Flag indicating if the end point was reached. */
    private boolean endPoint = false;
    
    /** Flag indicating if robot position data is recommended in imagespace. */
    private boolean imageSpace = false;
    
  //**************************Components*********************/
    /** Represents the distance to a virtual fixture in millimeters. */
    private double distance;

    /** The logging object for logging output.*/
    private ITaskLogger log = new DummyLogger();
    
    /**
     * Field containing the last NUM_FORCE_NORM_VECTORS norm vectors of the
     * direction of force. This field is used to average the force direction for
     * the cone to avoid fast changes in direction of introduced force.
     */
    private Vector[] lastNormVectorsForce = new Vector[NUM_FORCE_NORM_VECTORS];

    /**
     * Transformation matrix containing the transformation from robot base
     * coordinate frame to the cone coordinate frame.
     */
    private MatrixTransformation transformationBaseToCone;

    /** Geometric type of active virtual fixture. */
    private VirtualFixtureType activeVirtualFixture = VirtualFixtureType.Plane;
    /**
     * Position of the virtual fixture in cartesian space. If the activ evirtaul
     * fixture is a cone it is equal to the position of the cone tip
     */
    private Vector virtualFixturePosition;

    /**
     * Norm vector of the active Virtual fixtures. For a plane this is the norm
     * vector of the plane and for cone it is the symmetry axis of the cone.
     */
    private Vector virtualFixtureNormVector;
    /** Opening angle of the cone. */
    private double virtualFixturePhi;
    /** Current //TODO @Sebastian comment on. */
    private Vector normVector = Vector.of(0, 0, 1);

  //*************************Parameters**********************/
    /** Distance-parameter in millimeters for fixture calculation. */
    private double awaredist = 20; 
    // TODO initialize in init() method or as constants.
    /**
     * Distance to a virtual fixture in millimeters from the calculation before.
     */
    private double lastDistance = 0.0; 


  //***************************Methods***********************/
    /**
     * In this Function control Mode Parameters are set and the commanded pose
     * are calculated due the current LWR State. During the VirtualFixtures
     * State the Cartesian Stiffness is set according to the distance towards
     * the Fixtures and the movement (approaching, removing), the
     * NullSpaceStiffness is set to zero and the Pose is set to the measured
     * pose.
     * 
     * @param lwrStatemachine
     *            The operated state machine
     * @see ILwrState
     */
    @Override
    public final void calcControlParam(final LwrStatemachine lwrStatemachine,
	    final RobotDataSet currentRobotDataSet) {
	int[] newStiffness = { 0, 0, 0, 0, 0, 0 };
	double aDampVal = 0.0, stiffVal = 0.0;
	if (lwrStatemachine.stateChanged) {

	    coneTip = false;
	    endPoint = false;
	    if (this.activeVirtualFixture == VirtualFixtureType.Cone) {
		awaredist = 10;
		lwrStatemachine.stateChanged = false;

	    } else {
		awaredist = 20;
		lwrStatemachine.stateChanged = false;
	    }
	}
	if (this.activeVirtualFixture == VirtualFixtureType.Plane) {
	    Vector dv = currentRobotDataSet.getCurPose().getTranslation().subtract(
		    virtualFixturePosition);
	    distance = virtualFixtureNormVector.dotProduct(dv);
	    normVector = virtualFixtureNormVector;

	} else if (this.activeVirtualFixture == VirtualFixtureType.Cone) {

	    Vector xAxis;
	    Vector yAxis;
	    double n;
	    if (virtualFixtureNormVector.getX() != 0) {
		n = -(virtualFixtureNormVector.getY() + virtualFixtureNormVector
			.getZ()) / virtualFixtureNormVector.getX();
		xAxis = Vector.of(n, 1, 1).normalize();
		yAxis = virtualFixtureNormVector.crossProduct(xAxis);
	    } else if (virtualFixtureNormVector.getY() != 0) {
		n = -(virtualFixtureNormVector.getX() + virtualFixtureNormVector
			.getZ()) / virtualFixtureNormVector.getY();
		yAxis = Vector.of(1, n, 1).normalize();
		xAxis = yAxis.crossProduct(virtualFixtureNormVector);
	    } else {
		xAxis = Vector.of(1, 0, 0);
		yAxis = Vector.of(0, 1, 0);
	    }

	    transformationBaseToCone = MatrixTransformation.of(
		    virtualFixturePosition,
		    Matrix.ofRowFirst(1, 0, 0, 0, 1, 0, 0, 0, 1));

	    Vector curCartPoseCOcone = transformationBaseToCone
		    .getRotationMatrix().multiply(
			    currentRobotDataSet.getCurPose().getTranslation().subtract(
				    virtualFixturePosition));

	    double diagonale = Math.sqrt(Math.pow(curCartPoseCOcone.getX(), 2)
		    + Math.pow(curCartPoseCOcone.getY(), 2));

	    distance = ((Math.sin((virtualFixturePhi) / 2)
		    * curCartPoseCOcone.getZ() - diagonale) * Math
		    .cos(virtualFixturePhi / 2));
	    double sign = Math.signum(distance);
	    Vector vTemp = curCartPoseCOcone.withZ(0);

	    double z = Math.abs(distance) * Math.sin(virtualFixturePhi / 2);

	    if (curCartPoseCOcone.length() < (awaredist / 2)
		    && curCartPoseCOcone.length() > CONE_TOLERANCE_ENDPOINT) {

		coneTip = true;
		normVector = curCartPoseCOcone.invert().normalize();
		distance = curCartPoseCOcone.length();
	    } else if (curCartPoseCOcone.length() <= CONE_TOLERANCE_ENDPOINT) {
		endPoint = true;
		normVector = curCartPoseCOcone.invert().normalize();
	    } else {
		if (sign == 1) {
		    vTemp = vTemp.invert().multiply(distance / diagonale);
		} else {
		    vTemp = vTemp.multiply(distance / diagonale);
		}
		vTemp = vTemp.withZ(Math.abs(z));
		MatrixTransformation TransformationConeToBase = transformationBaseToCone
			.invert();
		normVector = TransformationConeToBase.getRotation()
			.applyTo(vTemp.normalize()).normalize();
	    }
	    if (lwrStatemachine.stateChanged) {

		for (int i = 0; i < NUM_FORCE_NORM_VECTORS; i++) {
		    lastNormVectorsForce[i] = normVector;
		}
	    }
	    for (int i = 1; i < NUM_FORCE_NORM_VECTORS; i++) {
		normVector = lastNormVectorsForce[i - 1]
			.add(lastNormVectorsForce[i]);
	    }
	    normVector = normVector.normalize();
	    for (int i = NUM_FORCE_NORM_VECTORS - 1; i >= 0; i--) {
		if (i == 0) {
		    lastNormVectorsForce[i] = normVector;
		} else {
		    lastNormVectorsForce[i] = lastNormVectorsForce[i - 1];
		}

	    }

	}
	if (distance < awaredist) {
	    aDampVal = 0.7;

	    if (coneTip) {
		if (endPoint) {
		    stiffVal = CART_TRANSL_STIFFNESS_MAX;
		} else {
		    stiffVal = getStiffnessValueConeTip(distance, awaredist);
		}
		lwrStatemachine.cmdPose = MatrixTransformation.of(
			virtualFixturePosition,
		currentRobotDataSet.getCurPose().getRotation());

	    } else if (distance >= 0) {

		if (distance - lastDistance <= 0) {
		    stiffVal = getStiffnessValueApproach(distance, awaredist);

		} else {
		    stiffVal = getStiffnessValueApproach(distance, awaredist);
		}
		lwrStatemachine.cmdPose = MatrixTransformation.of(
			currentRobotDataSet.getCurPose().getTranslation(),
			currentRobotDataSet.getCurPose().getRotation());
	    } else {
		stiffVal = CART_TRANSL_STIFFNESS_MAX;
		lwrStatemachine.cmdPose = MatrixTransformation.of(
			currentRobotDataSet.getCurPose().getTranslation().subtract(
				normVector.multiply(distance)),
				currentRobotDataSet.getCurPose().getRotation());
	    }
	} else {
	    stiffVal = 0.01;
	    aDampVal = 0.7;
	    lwrStatemachine.cmdPose = MatrixTransformation.of(
		    currentRobotDataSet.getCurPose().getTranslation(),
		    currentRobotDataSet.getCurPose().getRotation());
	}
	Vector aTransStiffVal;
	if (coneTip) {
	    aTransStiffVal = Vector.of(stiffVal, stiffVal, stiffVal);
	} else {
	    aTransStiffVal = Vector.of(Math.abs(normVector.getX() * stiffVal),
		    Math.abs(normVector.getY() * stiffVal),
		    Math.abs(normVector.getZ() * stiffVal));
	}
	double aRotStiffVal = stiffVal * 150 / 5000;
	double aNullStiffVal = 0;

	lastDistance = distance;

	// We are in CartImp Mode,
	// Modify the settings:
	// NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
	// NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
	// WILL DESTABILIZE THE CONTROLLER
	CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;

	cartImp.parametrize(CartDOF.X).setStiffness(aTransStiffVal.getX());
	cartImp.parametrize(CartDOF.Y).setStiffness(aTransStiffVal.getY());
	cartImp.parametrize(CartDOF.Z).setStiffness(aTransStiffVal.getZ());
	cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
	newStiffness[0] = (int) aTransStiffVal.getX();
	newStiffness[1] = (int) aTransStiffVal.getY();
	newStiffness[2] = (int) aTransStiffVal.getZ();
	newStiffness[3] = (int) aRotStiffVal;
	newStiffness[4] = (int) aRotStiffVal;
	newStiffness[5] = (int) aRotStiffVal;
	cartImp.setNullSpaceStiffness(aNullStiffVal);
	cartImp.parametrize(CartDOF.ALL).setDamping(aDampVal);
	// Send the new Stiffness settings down to the
	// controller
	lwrStatemachine.controlMode = cartImp;
	lwrStatemachine.curCartStiffness = newStiffness;
    }

    /**
     * In this Function the Stiffness value for the case that the robot is
     * getting closer to a virtual fixture is calculated.
     * 
     * @param dist
     *            the minimum distance to the Virtual Fixture
     * @param awareDistance
     *            the minimum distance from where on the stiffness is changed
     * @return the calculated stiffness value.
     */

    public final double getStiffnessValueApproach(final double dist,
	    final double awareDistance) {
	/*
	 * Calculation of the damping parameter when approaching the boundary.
	 */

	if (dist >= awareDistance) {
	    return 0.01;
	} else if (dist < awareDistance && dist > 0) {
	    double y;
	    double m = CART_TRANSL_STIFFNESS_MAX / Math.pow(awareDistance, 2);
	    y = m * Math.pow(awareDistance - dist, 2);
	    return y;
	} else {
	    return CART_TRANSL_STIFFNESS_MAX;
	}
    }

    /**
     * In this Function the Stiffness value for the case that the robot is
     * getting further away from a virtual fixture is calculated.
     * 
     * @param dist
     *            the minimum distance to the Virtual Fixture
     * @param awareDistance
     *            the minimum distance from where on the stiffness is changed
     * @return the calculated stiffness value.
     */
    public final double getStiffnessValueConeTip(final double dist,
	    final double awareDistance) {

	double max = CART_TRANSL_STIFFNESS_MAX / 2;
	if (dist < awareDistance && dist > 0) {
	    double y;
	    double m = max / Math.pow(awareDistance, 3);
	    y = CART_TRANSL_STIFFNESS_MAX - m
		    * Math.pow(awareDistance - dist, 3);
	    return y;
	} else {
	    return max;
	}
    }

    /**
     * In this Function the Stiffness value for the case that the robot is
     * getting further away from a virtual fixture is calculated.
     * 
     * @param dist
     *            the minimum distance to the Virtual Fixture
     * @param awareDistance
     *            the minimum distance from where on the stiffness is changed
     * @return the calculated stiffness value.
     */
    public final double getStiffnessValueRemoved(final double dist,
	    final double awareDistance) {

	double zeroPoint = awareDistance / 2;
	if (dist >= zeroPoint) {
	    return 0.01;
	} else if (dist < zeroPoint && dist > 0) {
	    double y;
	    double m = CART_TRANSL_STIFFNESS_MAX / Math.pow(zeroPoint, 3);
	    y = m * Math.pow(zeroPoint - dist, 3);
	    return y;
	} else {
	    return CART_TRANSL_STIFFNESS_MAX;
	}
    }

    /**
     * In this Function the CommandIGTmessage which is received from the State
     * Control is read and interpreted due to the Current State and if requested
     * and allowed the State is Changed. Furthermore the parameter of the state
     * are set and the class of the LWRState Object is changed according to the
     * state change In The VirtualFixtures State the allowed state transitions
     * are:<br>
     * - IDLE (transition condition: none)<br>
     * - GravComp (transition condition: none)<br>
     * - PathImp (transition condition: RegistrationFinished AND DataSend AND
     * TransformRecieved)<br>
     * - MoveToPose (transition condition: RegistrationFinished AND DataSend AND
     * TransformRecieved)<br>
     * If a transition request to another State is requested or the number of
     * Parameters is incorrect, the state is set to LWRError and the Error code
     * is set to 2= Transition not allowed.
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

	    lwrStatemachine.stateChanged = true;
	    if (cmdArray[1].contentEquals("img")) {
		this.imageSpace = true;
	    } else if (cmdArray[1].contentEquals("rob")) {
		this.imageSpace = false;
	    } else {
		lwrStatemachine.ErrorMessage = ("Unexpected "
			+ "coordinate system (supported are img or plane)");
		lwrStatemachine.ErrorFlag = true;
	    }
	    if (cmdArray[2].contentEquals("plane")) {
		this.activeVirtualFixture = VirtualFixtureType.Plane;
	    } else if (cmdArray[2].contentEquals("cone")) {
		this.activeVirtualFixture = VirtualFixtureType.Cone;
	    } else {
		lwrStatemachine.ErrorMessage = ("Unexpected "
			+ "VF type (supported are plane or cone)");
		lwrStatemachine.ErrorFlag = true;
	    }

	    this.virtualFixturePosition = Vector.of(
		    Double.parseDouble(cmdArray[3]),
		    Double.parseDouble(cmdArray[4]),
		    Double.parseDouble(cmdArray[5]));
	    this.virtualFixtureNormVector = Vector.of(
		    Double.parseDouble(cmdArray[6]),
		    Double.parseDouble(cmdArray[7]),
		    Double.parseDouble(cmdArray[8]));

	    if (this.imageSpace && cmdPacket.isTransformReceived()) {
		this.virtualFixturePosition = cmdPacket.getTrafo()
			.applyTo(this.virtualFixturePosition);
		this.virtualFixtureNormVector = cmdPacket.getTrafo()
			.applyTo(this.virtualFixtureNormVector);

	    }
	    if (this.activeVirtualFixture == VirtualFixtureType.Cone) {

		this.virtualFixturePhi = Math.toRadians(135.0); // TODO @Sebastian constant?
	    }
	    System.out.println("Virtaul Fixture ( ap ="
		    + this.virtualFixturePosition + ", n "
		    + this.virtualFixtureNormVector + ", type " + cmdArray[2]
		    + ") is now active!");

	} else {
	    lwrStatemachine.ErrorCode = OpenIGTLinkErrorCode.IllegalInstruction;
	    lwrStatemachine.ErrorMessage = "Unexpected Messagetype recieved! Expected STRING";
	}
    }

    /**
     * In this Function the Acknowledge IGTMessage which is send to the State
     * Control is defined due the current LWR State. In the Virtual Fixtures
     * State IGTString is Set to
     * "VirtualFixtures;plane;"/"VirtualFixtures;cone;" or
     * "VirtualFixtures;none;" dependent on the selected Virtual Fixtures type.
     * 
     * @param lwrStatemachine
     *            The operated Statemachine
     */
    @Override
    public final void setAckPacket(final LwrStatemachine lwrStatemachine) {
	String ack;
	if (this.activeVirtualFixture == VirtualFixtureType.Plane) {
	    if (distance >= awaredist) {
		ack = "VirtualFixtures;plane;0;";
	    } else if (distance < awaredist && distance > 0) {
		ack = "VirtualFixtures;plane;1;";
	    } else {
		ack = "VirtualFixtrues;plane;2;";
	    }
	} else if (this.activeVirtualFixture == VirtualFixtureType.Cone) {
	    if (distance >= awaredist) {
		ack = "VirtualFixtures;cone;0;";
	    } else if (distance < awaredist && distance > 0) {
		ack = "VirtualFixtures;cone;1;";
	    } else {
		ack = "VirtualFixtrues;cone;2;";
	    }
	} else {
	    ack = "VirtualFixtures;none;";
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
