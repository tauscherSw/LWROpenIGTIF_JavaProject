/*=========================================================================

  Program:   VisualSenderData
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

package de.uniHannover.imes.igtIf.communication.visualization;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

/**
 * This class describes the data, send to the slicer-visualization by the
 * visualization-communication-interface.
 */
final class VisualSenderData {
    
    //**************************Components*********************/
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


    
    //***************************Methods***********************/
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
