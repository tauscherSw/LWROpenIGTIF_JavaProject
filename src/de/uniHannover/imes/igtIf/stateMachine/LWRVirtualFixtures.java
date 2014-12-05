package de.uniHannover.imes.igtIf.stateMachine;

import java.io.UnsupportedEncodingException;
import java.nio.ByteBuffer;

import org.medcare.igtl.messages.StringMessage;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

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

public class LWRVirtualFixtures implements ILWRState {

    public Vector VirtualFixture_ap;
    public Vector VirtualFixture_n;
    public double VirtualFixture_phi;
    public int VFtype = 0;
    public boolean ImageSpace = false;
    // Some Variables for Stiffness Control
    private double distance;
    private double awaredist = 50;
    private double last_distance = 0.0;
    private double StiffVal = 0.0;
    private MatrixTransformation T_Base_cone;
    private Vector normvec;
    int i = 0;

    boolean ConeTip = false;
    boolean EndPoint = false;

    /**
     * In this Function the Stiffness value for the case that the robot is
     * getting closer to a virtual fixture is calculated
     * 
     * @param dist
     *            the minimum distance to the Virtual Fixture
     * @param awaredist
     *            the minimum distance from where on the stiffness is changed
     */

    public double get_stiffness_value_approach(double dist, double awaredist) // Berechnung
									      // der
									      // D�mpfung
									      // bei
									      // Ann�herung
									      // an
									      // die
									      // Grenze
    {
	double max_stiff = 5000;
	if (dist >= awaredist)
	    return 0.01;
	else if (dist < awaredist && dist > 0) {
	    double y;
	    double m = max_stiff / Math.pow(awaredist, 2);
	    y = m * Math.pow(awaredist - dist, 2);
	    return y;
	} else
	    return max_stiff;
    }

    /**
     * In this Function the Stiffness value for the case that the robot is
     * getting further away from a virtual fixture is calculated
     * 
     * @param dist
     *            the minimum distance to the Virtual Fixture
     * @param awaredist
     *            the minimum distance from where on the stiffness is changed
     */
    public double get_stiffness_value_remove(double dist, double awaredist) {

	double zero_point = awaredist / 2;
	double max_stiff = 5000;
	if (dist >= zero_point)
	    return 0.01;
	else if (dist < zero_point && dist > 0) {
	    double y;
	    double m = max_stiff / Math.pow(zero_point, 3);
	    y = m * Math.pow(zero_point - dist, 3);
	    return y;
	} else
	    return max_stiff;
    }

    /**
     * In this Function the Stiffness value for the case that the robot is
     * getting further away from a virtual fixture is calculated
     * 
     * @param dist
     *            the minimum distance to the Virtual Fixture
     * @param awaredist
     *            the minimum distance from where on the stiffness is changed
     */
    public double get_stiffness_value_ConeTip(double dist, double awaredist) {

	double max_stiff = 2500;
	if (dist < awaredist && dist > 0) {
	    double y;
	    double m = max_stiff / Math.pow(awaredist, 3);
	    y = 5000 - m * Math.pow(awaredist - dist, 3);
	    return y;
	} else
	    return max_stiff;
    }

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
     * @see ILWRState
     */
    @Override
    public void CalcControlParam(LWRStatemachine lwrStatemachine) {

	double aDampVal = 0.0, StiffVal = 0.0;
	double kr = 50;
	if (lwrStatemachine.InitFlag) {
	    lwrStatemachine.InitFlag = false;
	    ConeTip = false;
	    EndPoint = false;
	}
	if (this.VFtype == 1) {
	    // Berechnung des Abstandsvektors zur Ebene
	    Vector dv = lwrStatemachine.curPose.getTranslation().subtract(
		    VirtualFixture_ap);
	    distance = VirtualFixture_n.dotProduct(dv);
	    normvec = VirtualFixture_n;

	    // Berechnung des Abstandsvektors zur Ebene
	} else if (VFtype == 2) {
	    // Transformieren der EE-Koordinaten in das (KS)_Kegel
	    double theta = Math.asin(VirtualFixture_n.getX());
	    double Psi = Math.asin(VirtualFixture_n.getZ() / Math.cos(theta));
	    Vector x_axis;
	    Vector y_axis;
	    double n;
	    if (VirtualFixture_n.getX() != 0) {
		n = -(VirtualFixture_n.getY() + VirtualFixture_n.getZ())
			/ VirtualFixture_n.getX();
		x_axis = Vector.of(n, 1, 1).normalize();
		y_axis = VirtualFixture_n.crossProduct(x_axis);
	    } else if (VirtualFixture_n.getY() != 0) {
		n = -(VirtualFixture_n.getX() + VirtualFixture_n.getZ())
			/ VirtualFixture_n.getY();
		y_axis = Vector.of(1, n, 1).normalize();
		x_axis = y_axis.crossProduct(VirtualFixture_n);
	    } else {
		x_axis = Vector.of(1, 0, 0);
		y_axis = Vector.of(0, 1, 0);
	    }
	    // T_Base_cone = MatrixTransformation.of(VirtualFixture_ap,
	    // Matrix.ofRowFirst(x_axis.getX(), x_axis.getY(), x_axis.getZ(),
	    // y_axis.getX(), y_axis.getY(), y_axis.getZ(),
	    // VirtualFixture_n.getX(), VirtualFixture_n.getY(),
	    // VirtualFixture_n.getZ()));
	    T_Base_cone = MatrixTransformation.of(VirtualFixture_ap,
		    Matrix.ofRowFirst(1, 0, 0, 0, 1, 0, 0, 0, 1));
	    // curCartPoseCOcone =
	    // T_Base_cone.applyTo(curMsrCartPose.getTranslation()).invert();
	    Vector curCartPoseCOcone = T_Base_cone.getRotationMatrix()
		    .multiply(
			    lwrStatemachine.curPose.getTranslation().subtract(
				    VirtualFixture_ap));
	    /*
	     * double dphi = Math.acos(
	     * curCartPoseCOcone.dotProduct(Vector.of(0,
	     * 0,1))/curCartPoseCOcone.length()) - VirtualFixture_phi/2 ; double
	     * psi=
	     * Math.atan2(curCartPoseCOcone.getY(),curCartPoseCOcone.getX());
	     * double l_xy= Math.sqrt(1-Math.pow(Math.sin(VirtualFixture_phi/2),
	     * 2)); Vector x_cone=Vector.of(l_xy*Math.sin(psi),
	     * l_xy*Math.cos(psi), Math.sin(VirtualFixture_phi/2)); double
	     * xconeLength =
	     * curCartPoseCOcone.dotProduct(x_cone)/(Math.cos(VirtualFixture_phi
	     * /2)*curCartPoseCOcone.length());
	     * normvec=curCartPoseCOcone.invert(
	     * ).add(x_cone.normalize().multiply(xconeLength));
	     * 
	     * normvec = normvec.normalize();
	     */

	    // Berechnung des Abstandes
	    double diagonale = Math.sqrt(Math.pow(curCartPoseCOcone.getX(), 2)
		    + Math.pow(curCartPoseCOcone.getY(), 2));

	    distance = ((Math.sin((VirtualFixture_phi) / 2)
		    * curCartPoseCOcone.getZ() - diagonale) * Math
		    .cos(VirtualFixture_phi / 2));
	    double sign = Math.signum(distance);
	    // distance = Math.abs(distance);
	    Vector v_temp = curCartPoseCOcone.withZ(0);
	    // Berechnung des Abstandsvektors
	    double z = Math.abs(distance) * Math.sin(VirtualFixture_phi / 2);

	    i++;

	    // Radius des Kegels ab dem gesperrt wird
	    /*
	     * double zg= kr/Math.tan(VirtualFixture_phi/2); //Sperrparameter
	     * double sg= 2*zg; double tiefe = zg-curCartPoseCOcone.getZ();
	     * double lr;
	     */

	    if (curCartPoseCOcone.length() < (awaredist)
		    && curCartPoseCOcone.length() > 10) {

		ConeTip = true;
		normvec = curCartPoseCOcone.invert().normalize();
		distance = curCartPoseCOcone.length();
	    } else if (curCartPoseCOcone.length() <= 10) {
		EndPoint = true;
	    } else {
		if (sign == 1) {
		    v_temp = v_temp.invert().multiply(distance / diagonale);
		} else {
		    v_temp = v_temp.multiply(distance / diagonale);
		}
		v_temp = v_temp.withZ(Math.abs(z));
		MatrixTransformation T_cone_Base = T_Base_cone.invert();
		normvec = T_cone_Base.getRotation().applyTo(v_temp.normalize())
			.normalize();
	    }

	    if (i >= 100) {
		System.out.println("cur Pose (COFcone) " + curCartPoseCOcone);
		System.out.println("normvec (COFcone) " + normvec);
		System.out.println("normvec (COFcone) " + v_temp);
		System.out.println("distance (COFcone) " + distance);
		System.out.println("z :" + z);
		i = 0;

	    }
	    // Hier stimmt etwas noch nicht!!!!
	}
	// setzen der Stellgr��en D und K
	if (distance < awaredist) {
	    aDampVal = 0.7;

	    if (ConeTip) {
		if (EndPoint) {
		    StiffVal = 5000;
		} else {
		    StiffVal = get_stiffness_value_ConeTip(distance, awaredist);
		}
		lwrStatemachine.cmdPose = MatrixTransformation.of(
			VirtualFixture_ap,
			lwrStatemachine.curPose.getRotation());

	    } else if (distance >= 0) // im zul�ssigen Bereich
	    {

		if (distance - last_distance <= 0) // EE n�hert sich an die
						   // Grenze
		{
		    StiffVal = get_stiffness_value_approach(distance, awaredist);

		} else // EE entfernt sich von der Grenze
		{
		    StiffVal = get_stiffness_value_remove(distance, awaredist);
		}
		lwrStatemachine.cmdPose = MatrixTransformation.of(
			lwrStatemachine.curPose.getTranslation(),
			lwrStatemachine.curPose.getRotation());
	    } else // im gesperrten Bereich
	    {
		StiffVal = 5000;
		lwrStatemachine.cmdPose = MatrixTransformation.of(
			lwrStatemachine.curPose.getTranslation().subtract(
				normvec.multiply(distance)),
			lwrStatemachine.curPose.getRotation());
	    }
	} else { // au�erhalb des Gefahrenbereichs
	    StiffVal = 0.01;
	    aDampVal = 0.7;
	    lwrStatemachine.cmdPose = MatrixTransformation.of(
		    lwrStatemachine.curPose.getTranslation(),
		    lwrStatemachine.curPose.getRotation());
	}
	Vector aTransStiffVal;
	if (EndPoint) {
	    aTransStiffVal = Vector.of(5000, 5000, 5000);
	} else {
	    aTransStiffVal = Vector.of(Math.abs(normvec.getX() * StiffVal),
		    Math.abs(normvec.getY() * StiffVal),
		    Math.abs(normvec.getZ() * StiffVal));
	}
	// Transformieren der Impedanzwerte in das Flanschkoordinatensystem
	double aRotStiffVal = StiffVal * 300 / 5000;
	double aNullStiffVal = StiffVal;

	// Vector TransStiffVal_tool =
	// lwrStatemachine.curPose.getRotation().getMatrix().multiply(aTransStiffVal);
	// if(TransStiffVal_tool.getX()<=0) TransStiffVal_tool.withX(0.01);
	// if(TransStiffVal_tool.getY()<=0) TransStiffVal_tool.withY(0.01);
	// if(TransStiffVal_tool.getZ()<=0) TransStiffVal_tool.withZ(0.01);

	last_distance = distance;

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
	cartImp.setNullSpaceStiffness(aNullStiffVal);
	// Send the new Stiffness settings down to the
	// controller
	lwrStatemachine.controlMode = cartImp;
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
    public void SetACKPacket(LWRStatemachine lwrStatemachine) {
	String ACK;
	// TODO Automatisch generierter Methodenstub
	if (VFtype == 1) {
	    ACK = "VirtualFixtrues;plane;";
	} else if (VFtype == 2) {
	    ACK = "VirtualFixtrues;cone;";
	} else {
	    ACK = "VirtualFixtrues;none;";
	}
	String ACKname = "ACK_" + String.valueOf(lwrStatemachine.UID);
	// Send the string to StateControl
	StringMessage String = new StringMessage(ACKname, ACK);
	lwrStatemachine.AckIGTmessage = String;// TODO Automatisch generierter
					       // Methodenstub
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
     * @see ILWRState
     */
    @Override
    public void InterpretCMDPacket(LWRStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub
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

		lwrStatemachine.InitFlag = true;
		if (CMD_Array[1].contentEquals("img")) {
		    this.ImageSpace = true;
		} else if (CMD_Array[1].contentEquals("rob")) {
		    this.ImageSpace = false;
		} else {
		    Error = true;
		    lwrStatemachine.ErrorMessage = ("Unexpected coordinate system (supported are img or plane)");
		    lwrStatemachine.ErrorFlag = true;
		}
		if (CMD_Array[2].contentEquals("plane")) {
		    this.VFtype = 1;
		} else if (CMD_Array[2].contentEquals("cone")) {
		    this.VFtype = 2;
		} else {
		    Error = true;
		    lwrStatemachine.ErrorMessage = ("Unexpected VF type (supported are plane or cone)");
		    lwrStatemachine.ErrorFlag = true;
		}

		this.VirtualFixture_ap = Vector.of(
			Double.parseDouble(CMD_Array[3]),
			Double.parseDouble(CMD_Array[4]),
			Double.parseDouble(CMD_Array[5]));
		this.VirtualFixture_n = Vector.of(
			Double.parseDouble(CMD_Array[6]),
			Double.parseDouble(CMD_Array[7]),
			Double.parseDouble(CMD_Array[8]));
		if (this.VFtype == 2) {
		    this.VirtualFixture_phi = Double.parseDouble(CMD_Array[9])
			    * Math.PI / 180;
		}
		System.out.println("Virtaul Fixture ( ap ="
			+ this.VirtualFixture_ap + ", n "
			+ this.VirtualFixture_n + ", type " + CMD_Array[2]
			+ ") is now active!");
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
