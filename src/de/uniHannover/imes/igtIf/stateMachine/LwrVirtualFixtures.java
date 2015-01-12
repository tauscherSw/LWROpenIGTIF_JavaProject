/*=========================================================================

  Program:   LWRVirtualFixtures
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
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine.OpenIGTLinkErrorCode;

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
 * @author Sebastian Tauscher
 * @version 0.1
 */

public class LwrVirtualFixtures implements ILwrState {

    /** Distance-parameter in millimeters for fixture calculation. */
    private double awaredist = 20; //TODO initialize in init() method or as constants.
    boolean coneTip = false; // TODO @Sebastian: Javadoc

    /** Represents the distance to a virtual fixture in millimeters. */
    private double distance;
    /** Flag indicating if the endpoint was reached. */
    private boolean endPoint = false;
    /** Flag indicating if robot position data is recommended in imagespace. */
    private boolean imageSpace = false;
    /**
     * Distance to a virtual fixture in millimeters from the calculation before.
     */
    private double last_distance = 0.0; //TODO initialize in init() method or as constants.
    
    /**
     * Maximum cartesian-translational stiffness of the lwr in N/m.
     */
    private static final double CART_TRANSL_STIFFNESS_MAX = 5000;

    private Vector normvec = Vector.of(0, 0, 1); //TODO @Sebastian: Javadoc and naming.
    private Vector normvec_new = Vector.of(0, 0, 1); //TODO @Sebastian: Javadoc and naming.
    private Vector normvec_old = Vector.of(0, 0, 1); //TODO @Sebastian: Javadoc and naming.
    private Vector normvec_old2 = Vector.of(0, 0, 1); //TODO @Sebastian: Javadoc and naming.
    private Vector normvec_old3 = Vector.of(0, 0, 1); //TODO @Sebastian: Javadoc and naming.
    private Vector normvec_old4 = Vector.of(0, 0, 1); //TODO @Sebastian: Javadoc and naming.
    private Vector normvec_old5 = Vector.of(0, 0, 1); //TODO @Sebastian: Javadoc and naming.
    
    private MatrixTransformation T_Base_cone; //TODO @Sebastian: Javadoc and naming.
    private int virtualFixtureType = 0; //TODO @Sebastian: Javadoc and naming.
    private Vector virtualFixtureAp; //TODO @Sebastian: Javadoc and naming.
    private Vector VirtualFixture_n; //TODO @Sebastian: Javadoc and naming.
    private double VirtualFixture_phi; //TODO @Sebastian: Javadoc and naming.

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
    public final void calcControlParam(final LwrStatemachine lwrStatemachine) {
	int[] newStiffness = { 0, 0, 0, 0, 0, 0 };
	double aDampVal = 0.0, stiffVal = 0.0;
	if (lwrStatemachine.InitFlag) {

	    coneTip = false;
	    endPoint = false;
	    if (this.virtualFixtureType == 2) {
		awaredist = 10;
		lwrStatemachine.InitFlag = false;

	    } else {
		awaredist = 20;
		lwrStatemachine.InitFlag = false;
	    }
	}
	if (this.virtualFixtureType == 1) {
	    Vector dv = lwrStatemachine.curPose.getTranslation().subtract(
		    virtualFixtureAp);
	    distance = VirtualFixture_n.dotProduct(dv);
	    normvec = VirtualFixture_n;

	} else if (virtualFixtureType == 2) {

	    Vector xAxis;
	    Vector yAxis;
	    double n;
	    if (VirtualFixture_n.getX() != 0) {
		n = -(VirtualFixture_n.getY() + VirtualFixture_n.getZ())
			/ VirtualFixture_n.getX();
		xAxis = Vector.of(n, 1, 1).normalize();
		yAxis = VirtualFixture_n.crossProduct(xAxis);
	    } else if (VirtualFixture_n.getY() != 0) {
		n = -(VirtualFixture_n.getX() + VirtualFixture_n.getZ())
			/ VirtualFixture_n.getY();
		yAxis = Vector.of(1, n, 1).normalize();
		xAxis = yAxis.crossProduct(VirtualFixture_n);
	    } else {
		xAxis = Vector.of(1, 0, 0);
		yAxis = Vector.of(0, 1, 0);
	    }

	    T_Base_cone = MatrixTransformation.of(virtualFixtureAp,
		    Matrix.ofRowFirst(1, 0, 0, 0, 1, 0, 0, 0, 1));

	    Vector curCartPoseCOcone = T_Base_cone.getRotationMatrix()
		    .multiply(
			    lwrStatemachine.curPose.getTranslation().subtract(
				    virtualFixtureAp));

	    double diagonale = Math.sqrt(Math.pow(curCartPoseCOcone.getX(), 2)
		    + Math.pow(curCartPoseCOcone.getY(), 2));

	    distance = ((Math.sin((VirtualFixture_phi) / 2)
		    * curCartPoseCOcone.getZ() - diagonale) * Math
		    .cos(VirtualFixture_phi / 2));
	    double sign = Math.signum(distance);
	    Vector vTemp = curCartPoseCOcone.withZ(0);

	    double z = Math.abs(distance) * Math.sin(VirtualFixture_phi / 2);

	    if (curCartPoseCOcone.length() < (awaredist / 2)
		    && curCartPoseCOcone.length() > 10) {

		coneTip = true;
		normvec_new = curCartPoseCOcone.invert().normalize();
		distance = curCartPoseCOcone.length();
	    } else if (curCartPoseCOcone.length() <= 10) {
		endPoint = true;
		normvec_new = curCartPoseCOcone.invert().normalize();
	    } else {
		if (sign == 1) {
		    vTemp = vTemp.invert().multiply(distance / diagonale);
		} else {
		    vTemp = vTemp.multiply(distance / diagonale);
		}
		vTemp = vTemp.withZ(Math.abs(z));
		MatrixTransformation T_cone_Base = T_Base_cone.invert();
		normvec_new = T_cone_Base.getRotation()
			.applyTo(vTemp.normalize()).normalize();
	    }
	    if (lwrStatemachine.InitFlag) {
		normvec_old = normvec_new;
		normvec_old2 = normvec_old;
		normvec_old3 = normvec_old2;
		normvec_old4 = normvec_old3;
		normvec_old5 = normvec_old4;
		lwrStatemachine.InitFlag = false;
	    }

	    normvec = normvec_new.add(normvec_old).add(normvec_old2)
		    .add(normvec_old3).add(normvec_old4).add(normvec_old5)
		    .normalize();
	    normvec_old5 = normvec_old4;
	    normvec_old4 = normvec_old3;
	    normvec_old3 = normvec_old2;
	    normvec_old2 = normvec_old;
	    normvec_old = normvec;

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
			virtualFixtureAp,
			lwrStatemachine.curPose.getRotation());

	    } else if (distance >= 0) {

		if (distance - last_distance <= 0) {
		    stiffVal = getStiffnessValueApproach(distance, awaredist);

		} else {
		    stiffVal = getStiffnessValueApproach(distance, awaredist);
		}
		lwrStatemachine.cmdPose = MatrixTransformation.of(
			lwrStatemachine.curPose.getTranslation(),
			lwrStatemachine.curPose.getRotation());
	    } else {
		stiffVal = CART_TRANSL_STIFFNESS_MAX;
		lwrStatemachine.cmdPose = MatrixTransformation.of(
			lwrStatemachine.curPose.getTranslation().subtract(
				normvec.multiply(distance)),
			lwrStatemachine.curPose.getRotation());
	    }
	} else {
	    stiffVal = 0.01;
	    aDampVal = 0.7;
	    lwrStatemachine.cmdPose = MatrixTransformation.of(
		    lwrStatemachine.curPose.getTranslation(),
		    lwrStatemachine.curPose.getRotation());
	}
	Vector aTransStiffVal;
	if (coneTip) {
	    aTransStiffVal = Vector.of(stiffVal, stiffVal, stiffVal);
	} else {
	    aTransStiffVal = Vector.of(Math.abs(normvec.getX() * stiffVal),
		    Math.abs(normvec.getY() * stiffVal),
		    Math.abs(normvec.getZ() * stiffVal));
	}
	double aRotStiffVal = stiffVal * 150 / 5000;
	double aNullStiffVal = 0;

	last_distance = distance;

	// We are in CartImp Mode,
	// Modify the settings:
	// NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
	// NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
	// WILL DESTABILIZE THE CONTROLLER
	CartesianImpedanceControlMode cartImp 
	    = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;

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
	    y = CART_TRANSL_STIFFNESS_MAX - m * Math.pow(awareDistance - dist, 3);
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
     * @see ILwrState
     */
    @Override
    public final void interpretCmdPacket(final LwrStatemachine lwrStatemachine) {

	if (lwrStatemachine.IGTLdatatype.equals("STRING")) {
	    String cmdString;

	    cmdString = lwrStatemachine.cmdIgtMsg;
	    lwrStatemachine.paramString = cmdString.substring(cmdString
		    .indexOf(";"));
	    String[] cmdArray = cmdString.split(";");

	    lwrStatemachine.InitFlag = true;
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
		this.virtualFixtureType = 1;
	    } else if (cmdArray[2].contentEquals("cone")) {
		this.virtualFixtureType = 2;
	    } else {
		lwrStatemachine.ErrorMessage = ("Unexpected "
			+ "VF type (supported are plane or cone)");
		lwrStatemachine.ErrorFlag = true;
	    }

	    this.virtualFixtureAp = Vector.of(Double.parseDouble(cmdArray[3]),
		    Double.parseDouble(cmdArray[4]),
		    Double.parseDouble(cmdArray[5]));
	    this.VirtualFixture_n = Vector.of(Double.parseDouble(cmdArray[6]),
		    Double.parseDouble(cmdArray[7]),
		    Double.parseDouble(cmdArray[8]));

	    if (this.imageSpace && lwrStatemachine.transformReceivedFlag) {
		this.virtualFixtureAp = lwrStatemachine.transfRobotImg
			.applyTo(this.virtualFixtureAp);
		this.VirtualFixture_n = lwrStatemachine.transfRobotImg
			.applyTo(this.VirtualFixture_n);

	    }
	    if (this.virtualFixtureType == 2) {

		this.VirtualFixture_phi = Math.toRadians(135.0); //TODO Constant?
	    }
	    System.out.println("Virtaul Fixture ( ap ="
		    + this.virtualFixtureAp + ", n " + this.VirtualFixture_n
		    + ", type " + cmdArray[2] + ") is now active!");

	} else {
	    lwrStatemachine.ErrorCode = OpenIGTLinkErrorCode.IllegalInstruction;
	    lwrStatemachine.ErrorMessage = 
		    "Unexpected Messagetype recieved! Expected STRING";
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
	if (virtualFixtureType == 1) {
	    if (distance >= awaredist) {
		ack = "VirtualFixtures;plane;0;";
	    } else if (distance < awaredist && distance > 0) {
		ack = "VirtualFixtures;plane;1;";
	    } else {
		ack = "VirtualFixtrues;plane;2;";
	    }
	} else if (virtualFixtureType == 2) {
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
	lwrStatemachine.ackIgtMsg = ack;
    }

}
