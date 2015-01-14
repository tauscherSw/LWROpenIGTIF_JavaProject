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

	 /**
    * Enum for the virtual fixtures type {Cone, plane }
    */
   private static enum VirtualFixtureType {
   /** The virtual fixture has a plane geometry. */
   Plane,
	/** The virtual fixture has a cone geometry */
	Cone,	
   }
   /** Distance-parameter in millimeters for fixture calculation. */
   private double awaredist = 20; //TODO initialize in init() method or as constants.
   /** Flag indicates if the robot is near the tip of the cone. Due to this variable
    *  the half sphere used as virtual fixture instead of the cone tip is used to 
    *  avoid stability issues. */
   boolean coneTip = false;

   /** Represents the distance to a virtual fixture in millimeters. */
   private double distance;
   /** Flag indicating if the end point was reached. */
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
   
   private static final int NUM_FORCE_NORM_VECTORS = 5;
   
   /** Field containing the last NUM_FORCE_NORM_VECTORS norm vectors of the direction of force. 
    * This field is used to average the force direction for the cone to avoid fast changes in 
    * direction of introduced force. */
   private Vector [] lastNormVectorsForce = new Vector [NUM_FORCE_NORM_VECTORS];
  
   
   /**Transformation matrix containing the transformation from robot base coordinate frame to the cone coordinate frame.*/
   private MatrixTransformation TransformationBaseToCone;
   
   /**   Geometric type of active virtual fixture   */
   private VirtualFixtureType activeVirtualFixture = VirtualFixtureType.Plane;
   /** Position of the virtual fixture in cartesian space. If the activ evirtaul fixture is a cone  it is equal to the position of the cone tip */
   private Vector virtualFixturePosition; 
   
   /** Norm vector of the active Virtual fixtures. For a plane this is the norm vector of the plane and for cone it is the symmetry axis of the cone.*/
   private Vector VirtualFixtureNormVector;
   /** Opening angle of the cone.*/
   private double VirtualFixturePhi; 
   /** Current */
   private Vector normVector = Vector.of(0, 0, 1);
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
	    if (this.activeVirtualFixture == VirtualFixtureType.Cone) {
		awaredist = 10;
		lwrStatemachine.InitFlag = false;

	    } else {
		awaredist = 20;
		lwrStatemachine.InitFlag = false;
	    }
	}
	if (this.activeVirtualFixture == VirtualFixtureType.Plane) {
	    Vector dv = lwrStatemachine.curPose.getTranslation().subtract(
	    		virtualFixturePosition);
	    distance = VirtualFixtureNormVector.dotProduct(dv);
	    normVector = VirtualFixtureNormVector;

	} else if (this.activeVirtualFixture == VirtualFixtureType.Cone) {

	    Vector xAxis;
	    Vector yAxis;
	    double n;
	    if (VirtualFixtureNormVector.getX() != 0) {
		n = -(VirtualFixtureNormVector.getY() + VirtualFixtureNormVector.getZ())
			/ VirtualFixtureNormVector.getX();
		xAxis = Vector.of(n, 1, 1).normalize();
		yAxis = VirtualFixtureNormVector.crossProduct(xAxis);
	    } else if (VirtualFixtureNormVector.getY() != 0) {
		n = -(VirtualFixtureNormVector.getX() + VirtualFixtureNormVector.getZ())
			/ VirtualFixtureNormVector.getY();
		yAxis = Vector.of(1, n, 1).normalize();
		xAxis = yAxis.crossProduct(VirtualFixtureNormVector);
	    } else {
		xAxis = Vector.of(1, 0, 0);
		yAxis = Vector.of(0, 1, 0);
	    }

	    TransformationBaseToCone = MatrixTransformation.of(virtualFixturePosition,
		    Matrix.ofRowFirst(1, 0, 0, 0, 1, 0, 0, 0, 1));

	    Vector curCartPoseCOcone = TransformationBaseToCone.getRotationMatrix()
		    .multiply(
			    lwrStatemachine.curPose.getTranslation().subtract(
			    		virtualFixturePosition));

	    double diagonale = Math.sqrt(Math.pow(curCartPoseCOcone.getX(), 2)
		    + Math.pow(curCartPoseCOcone.getY(), 2));

	    distance = ((Math.sin((VirtualFixturePhi) / 2)
		    * curCartPoseCOcone.getZ() - diagonale) * Math
		    .cos(VirtualFixturePhi / 2));
	    double sign = Math.signum(distance);
	    Vector vTemp = curCartPoseCOcone.withZ(0);

	    double z = Math.abs(distance) * Math.sin(VirtualFixturePhi / 2);

	    if (curCartPoseCOcone.length() < (awaredist / 2)
		    && curCartPoseCOcone.length() > 10) {

		coneTip = true;
		normVector = curCartPoseCOcone.invert().normalize();
		distance = curCartPoseCOcone.length();
	    } else if (curCartPoseCOcone.length() <= 10) {
		endPoint = true;
		normVector = curCartPoseCOcone.invert().normalize();
	    } else {
		if (sign == 1) {
		    vTemp = vTemp.invert().multiply(distance / diagonale);
		} else {
		    vTemp = vTemp.multiply(distance / diagonale);
		}
		vTemp = vTemp.withZ(Math.abs(z));
		MatrixTransformation TransformationConeToBase = TransformationBaseToCone.invert();
		normVector = TransformationConeToBase.getRotation()
			.applyTo(vTemp.normalize()).normalize();
	    }
	    if (lwrStatemachine.InitFlag) {
	    	for(int i = 0; i<NUM_FORCE_NORM_VECTORS; i++){
	    		lastNormVectorsForce[i] = normVector;
	    	}
	    }
	    	for(int i = 1; i<NUM_FORCE_NORM_VECTORS; i++){
	    			normVector = lastNormVectorsForce[i-1].add(lastNormVectorsForce[i]);
	    	}
	    	normVector = normVector.normalize();
	    	for(int i = NUM_FORCE_NORM_VECTORS -1; i>= 0; i--){
	    		if(i == 0){
	    			lastNormVectorsForce[i] = normVector;
	    		}else{
	    			lastNormVectorsForce[i]= lastNormVectorsForce[i-1];
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
				normVector.multiply(distance)),
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
	    aTransStiffVal = Vector.of(Math.abs(normVector.getX() * stiffVal),
		    Math.abs(normVector.getY() * stiffVal),
		    Math.abs(normVector.getZ() * stiffVal));
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
	    	this.activeVirtualFixture = VirtualFixtureType.Plane;
	    } else if (cmdArray[2].contentEquals("cone")) {
	    	this.activeVirtualFixture = VirtualFixtureType.Cone;
	    } else {
		lwrStatemachine.ErrorMessage = ("Unexpected "
			+ "VF type (supported are plane or cone)");
		lwrStatemachine.ErrorFlag = true;
	    }

	    this.virtualFixturePosition = Vector.of(Double.parseDouble(cmdArray[3]),
		    Double.parseDouble(cmdArray[4]),
		    Double.parseDouble(cmdArray[5]));
	    this.VirtualFixtureNormVector = Vector.of(Double.parseDouble(cmdArray[6]),
		    Double.parseDouble(cmdArray[7]),
		    Double.parseDouble(cmdArray[8]));

	    if (this.imageSpace && lwrStatemachine.transformReceivedFlag) {
		this.virtualFixturePosition = lwrStatemachine.transfRobotImg
			.applyTo(this.virtualFixturePosition);
		this.VirtualFixtureNormVector = lwrStatemachine.transfRobotImg
			.applyTo(this.VirtualFixtureNormVector);

	    }
	    if (this.activeVirtualFixture == VirtualFixtureType.Cone) {

	    	this.VirtualFixturePhi = Math.toRadians(135.0); //TODO Constant?
	    }
	    System.out.println("Virtaul Fixture ( ap ="
		    + this.virtualFixturePosition + ", n " + this.VirtualFixtureNormVector
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
	lwrStatemachine.ackIgtMsg = ack;
   }

}
