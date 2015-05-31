package de.uniHannover.imes.igtIf.util;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

/**
 * Utility class for kinematic calculations of lwr iiwa.
 *
 */
public final class KinematicsLwrUtil {

    // **************************Constants**********************/
    /**
     * Represents the d parameter of the Denavit-Hartenberg robot
     * representation. It is the distance between on the perpendicular line of
     * two joints in millimeters.
     */
    private static final double[] DH_D_PARAMETER_LWRIIWA7 = new double[] { 160,
	    180, 180, 220, 180, 220, 80, 50 };

    // TODO do we have the parameters of iiwa 14?

    // *************************Constructors********************/
    /**
     * Privatized constructor, because this utility-class shouldn't be
     * instantiated.
     */
    private KinematicsLwrUtil() {
	super();
    }

    // ***************************Methods***********************/

    /**
     * In this function the homogeneous Matrix-Transformation for each Joint is
     * calculated from the set of denavit hartenberg parameter.
     * 
     * @param q
     *            The joint position for calculation.
     * @return the composed array of {@link MatrixTransformation} to each joint
     *         center point and to the flange. All transformations are
     *         calculation in relatively to the robots base.
     */
    public static MatrixTransformation[] calcDirectKinematic(
	    final JointPosition q) {
	MatrixTransformation trafoBaseToJoint1 = MatrixTransformation.of(Vector
		.of(0, 0, DH_D_PARAMETER_LWRIIWA7[0]), Matrix.ofRowFirst(

	Math.cos(q.get(0)), -Math.sin(q.get(0)), 0, Math.sin(q.get(0)),
		Math.cos(q.get(0)), 0, 0, 0, 1));

	MatrixTransformation trafoJoint1ToJoint2 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[1]), Matrix.ofRowFirst(
			Math.cos(q.get(1)), 0, Math.sin(q.get(1)), 0, 1, 0,
			-Math.sin(q.get(1)), 0, Math.cos(q.get(1))));

	MatrixTransformation trafoJoint2ToJoint3 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[2]), Matrix.ofRowFirst(
			Math.cos(q.get(2)), -Math.sin(q.get(2)), 0,
			Math.sin(q.get(2)), Math.cos(q.get(2)), 0, 0, 0, 1));

	MatrixTransformation trafoJoint3ToJoint4 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[3]), Matrix.ofRowFirst(
			Math.cos(-q.get(3)), 0, Math.sin(-q.get(3)), 0, 1, 0,
			-Math.sin(-q.get(3)), 0, Math.cos(-q.get(3))));

	MatrixTransformation trafoJoint4ToJoint5 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[4]), Matrix.ofRowFirst(
			Math.cos(q.get(4)), -Math.sin(q.get(4)), 0,
			Math.sin(q.get(4)), Math.cos(q.get(4)), 0, 0, 0, 1));

	MatrixTransformation trafoJoint5ToJoint6 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[5]), Matrix.ofRowFirst(
			Math.cos(q.get(5)), 0, Math.sin(q.get(5)), 0, 1, 0,
			-Math.sin(q.get(5)), 0, Math.cos(q.get(5))));

	MatrixTransformation trafoJoint6ToJoint7 = MatrixTransformation.of(
		Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[6]), Matrix.ofRowFirst(
			Math.cos(q.get(6)), -Math.sin(q.get(6)), 0,
			Math.sin(q.get(6)), Math.cos(q.get(6)), 0, 0, 0, 1));

	MatrixTransformation trafoJoint7ToEndeffector = MatrixTransformation
		.of(Vector.of(0, 0, DH_D_PARAMETER_LWRIIWA7[7]),
			Matrix.IDENTITY);
	MatrixTransformation[] retVal = new MatrixTransformation[8];
	retVal[0] = trafoBaseToJoint1;
	retVal[1] = retVal[0].compose(trafoJoint1ToJoint2);
	retVal[2] = retVal[1].compose(trafoJoint2ToJoint3);
	retVal[3] = retVal[2].compose(trafoJoint3ToJoint4);
	retVal[4] = retVal[3].compose(trafoJoint4ToJoint5);
	retVal[5] = retVal[4].compose(trafoJoint5ToJoint6);
	retVal[6] = retVal[5].compose(trafoJoint6ToJoint7);
	retVal[7] = retVal[6].compose(trafoJoint7ToEndeffector);
	return retVal;

    }

}
