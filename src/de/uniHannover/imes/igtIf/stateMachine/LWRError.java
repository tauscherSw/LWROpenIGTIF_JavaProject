package de.uniHannover.imes.igtIf.stateMachine;

import java.io.UnsupportedEncodingException;

import org.medcare.igtl.messages.StatusMessage;
import org.medcare.igtl.messages.TransformMessage;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * The LWRError is a save State the robots is automatically set in if an error
 * occurs. In this state the cartesian stiffness is set to a value of 100 and a
 * moving Virtual plane is defined in the xy-plane of the robotbase
 * coordinatesystem. Thereby only movements with an increasing z Value are
 * possible.
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */
public class LWRError implements LWRState {
    private Vector safetyplane_ap;
    private Vector safetyplane_n;
    private double safetydist = 30.0;
    private double last_distance;
    private Vector curPosition = null;
    private Vector lastPosition = Vector.of(0, 0, -100);

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
    public double get_stiffness_value_remove(double dist, double awaredist) // Berechnung
									    // der
									    // D�mpfung
									    // bei
									    // Entfernung
									    // von
									    // der
									    // Grenze
    {

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

    @Override
    public void CalcControlParam(LWRStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub
	curPosition = lwrStatemachine.curPose.getTranslation();
	double distance;
	double aDampVal = 0.0, StiffVal = 0.0, aRotStiffVal = 10.0, aNullStiffVal = 0.0;
	Vector aTransStiffVal = Vector.of(1000, 1000, 1000);
	// We are in CartImp Mode,
	// Modify the settings:
	// NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
	// NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
	// WILL DESTABILIZE THE CONTROLLER
	if (curPosition.getZ() > lastPosition.getZ()) {
	    safetyplane_ap = Vector.of(0.0, 0.0, curPosition.getZ() - 30);
	    safetyplane_n = Vector.of(0.0, 0.0, 1);
	    lastPosition = curPosition;
	}
	Vector dv = curPosition.subtract(safetyplane_ap);
	distance = safetyplane_n.dotProduct(dv);
	Vector normvec = safetyplane_n;
	aDampVal = 0.0;
	StiffVal = 0.0;
	if (lwrStatemachine.InitFlag) {
	    last_distance = distance;
	    lwrStatemachine.InitFlag = false;
	}
	// setzen der Stellgr��en D und K
	if (distance < safetydist) {
	    aDampVal = 0.7;

	    if (distance >= 0) // im zul�ssigen Bereich
	    {

		if (distance - last_distance <= 0) // EE n�hert sich an die
						   // Grenze
		{
		    StiffVal = get_stiffness_value_approach(distance,
			    safetydist);

		} else // EE entfernt sich von der Grenze
		{
		    StiffVal = get_stiffness_value_remove(distance, safetydist);
		}
		lwrStatemachine.cmdPose = MatrixTransformation.of(curPosition,
			lwrStatemachine.curPose.getRotation());
	    } else // im gesperrten Bereich
	    {
		StiffVal = 5000;
		lwrStatemachine.cmdPose = MatrixTransformation.of(
			curPosition.subtract(normvec.multiply(distance)),
			lwrStatemachine.curPose.getRotation());
	    }
	} else { // au�erhalb des Gefahrenbereichs
	    StiffVal = 100.0;
	    aDampVal = 0.7;
	    lwrStatemachine.cmdPose = MatrixTransformation.of(curPosition,
		    lwrStatemachine.curPose.getRotation());
	}
	aTransStiffVal = Vector.of(Math.abs(normvec.getX() * StiffVal),
		Math.abs(normvec.getY() * StiffVal),
		Math.abs(normvec.getZ() * StiffVal));
	// Transformieren der Impedanzwerte in das Flanschkoordinatensystem
	aRotStiffVal = StiffVal * 300 / 5000;
	aNullStiffVal = StiffVal;

	CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;
	cartImp.parametrize(CartDOF.X).setStiffness(aTransStiffVal.getX());
	cartImp.parametrize(CartDOF.Y).setStiffness(aTransStiffVal.getY());
	cartImp.parametrize(CartDOF.Z).setStiffness(aTransStiffVal.getZ());
	cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
	cartImp.setNullSpaceStiffness(aNullStiffVal);

	lwrStatemachine.controlMode = cartImp;
	lwrStatemachine.cmdPose = lwrStatemachine.curPose;
    }

    @Override
    public void SetACKPacket(LWRStatemachine lwrStatemachine) {
	// TODO Automatisch generierter Methodenstub
	String ACKname = "ACK_" + String.valueOf(lwrStatemachine.UID);
	// Send the string to StateControl
	StatusMessage Status = new StatusMessage(ACKname,
		lwrStatemachine.ErrorCode, 0, lwrStatemachine.ErrorMessage);
	lwrStatemachine.AckIGTmessage = Status;
    }

    /**
     * In this Function the Command String which is received from the State
     * Control is read and interpreted due to the Current State and if requested
     * and allowed the State is Changed. In The VirtualFixtures State the
     * allowed state transitions are:<br>
     * - IDLE (transition condition: none)<br>
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
