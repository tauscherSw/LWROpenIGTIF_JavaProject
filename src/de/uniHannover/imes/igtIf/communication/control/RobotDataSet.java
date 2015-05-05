package de.uniHannover.imes.igtIf.communication.control;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

/**
 * Represents a cyclic dataset of the robot.
 *
 */
public class RobotDataSet {

    /**
     * Current Joint Position of the LWR received via SmartServo.
     */
    private JointPosition curJntPose;
    /**
     * The current pose in Cartesian space of the LWR in robot coordinates.
     */
    private MatrixTransformation curPose;

    /**
     * Vector containing the force estimated at the tool center point by the
     * internal torque sensors.
     */
    private Vector tcpForce;

    /**
     * Constructs a new robot dataset by the given parameters.
     * 
     * @param jointPosition
     *            the joint position of the robot.
     * @param trafo
     *            the homogenous transformation matrix to the robot's
     *            end-effector.
     * @param extForce
     *            the externl force applied to the robots end-effector.
     */
    public RobotDataSet(final JointPosition jointPosition,
	    final MatrixTransformation trafo, final Vector extForce) {
	curJntPose = jointPosition;
	curPose = trafo;
	tcpForce = extForce;
    }

    /**
     * Getter for the joint position of the robot.
     * @return the curJntPose the joint position of the robot.
     */
    public final JointPosition getCurJntPose() {
	return curJntPose;
    }

    /**
     * Getter for the matrix transformation to the end-effector.
     * @return the matrix transformation to the end-effector.
     */
    public final MatrixTransformation getCurPose() {
	return curPose;
    }

    /**
     * Getter for the tcp force vector.
     * @return the tcp force vector.
     */
    public final Vector getTcpForce() {
	return tcpForce;
    }
}
