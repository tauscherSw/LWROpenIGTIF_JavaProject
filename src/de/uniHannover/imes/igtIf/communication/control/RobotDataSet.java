/*=========================================================================

  Program:   RobotDataSet
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
