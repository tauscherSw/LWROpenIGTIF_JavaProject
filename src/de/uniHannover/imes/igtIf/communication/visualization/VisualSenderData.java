package de.uniHannover.imes.igtIf.communication.visualization;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

/**
 * This class describes the data, send to the slicer-visualization by the
 * visualization-communication-interface.
 */
final class VisualSenderData {

    /**
     * Joint position of each robot joint in rad.
     */
    private JointPosition jntPos;
    /**
     * External force vector applied to the tcp.
     */
    private Vector tcpForce;
    /**
     * Cartesian pose of the tcp related to the robot's base.
     */
    private MatrixTransformation cartPoseTcpRobotBase;

    /**
     * Cartesian pose of the tcp related to an external base.
     */
    private MatrixTransformation cartPoseTcpExternalBase;

    /**
     * Getter for the joint position in rad.
     * 
     * @return the joint position in rad.
     */
    JointPosition getJntPos() {
	return jntPos;
    }

    /**
     * Getter for the external tcp force in N.
     * 
     * @return the external tcp force object.
     */
    Vector getTcpForce() {
	return tcpForce;
    }

    /**
     * Getter for the cartesian pose of the tcp in the robot's base.
     * 
     * @return the cartesian pose object.
     */
    MatrixTransformation getCartPoseRobotBase() {
	return cartPoseTcpRobotBase;
    }
    
    /**
     * Getter for the cartesian pose of the tcp in an external base.
     * 
     * @return the cartesian pose object.
     */
    MatrixTransformation getCartPoseTcpExternalBase() {
        return cartPoseTcpExternalBase;
    }

    /**
     * Sets the relevant data at once.
     * 
     * @param jointPosition
     *            the current joint position of the robot.
     * @param toolCenterPointForce
     *            the current tcp force of the robot.
     * @param cartesianPoseTcp
     *            the cartesian pose of the tcp of the robot relatively to the
     *            robot's base.
     * @param cartesianPoseTcpExternalBase
     *            the cartesian pose of the tcp of the robot relatively to an
     *            external base base.
     */
    void setData(final JointPosition jointPosition,
	    final Vector toolCenterPointForce,
	    final MatrixTransformation cartesianPoseTcp,
	    final MatrixTransformation cartesianPoseTcpExternalBase) {

	this.jntPos = jointPosition;
	this.tcpForce = toolCenterPointForce;
	this.cartPoseTcpRobotBase = cartesianPoseTcp;
	this.cartPoseTcpExternalBase = cartesianPoseTcpExternalBase;
    }


}
