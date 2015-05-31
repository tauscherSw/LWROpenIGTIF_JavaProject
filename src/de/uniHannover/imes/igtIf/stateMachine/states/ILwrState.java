/*=========================================================================

  Program:   LWRState
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher, Institute of Mechatronics Systems, Leibniz Universit�t Hannover. All rights reserved.

	
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

package de.uniHannover.imes.igtIf.stateMachine.states;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine;
import de.uniHannover.imes.igtIf.communication.control.CommandPacket;
import de.uniHannover.imes.igtIf.communication.control.RobotDataSet;

/**
 * Interface for the LWR States defining the Functions void
 * CalcControlParam(LWRStatemachine lwrStatemachine), void
 * SetACKString(LWRStatemachine lwrStatemachine) and void
 * SetACKString(LWRStatemachine lwrStatemachine).So far realized States are:
 * Idle, GravComp, Registration, SendData, WaitforData, Virtual Fixtures,
 * PathImp, MovetoPose and Error. For Further Information see the Documentation
 * of the particular states.
 * 
 * <pre>
 * How To add a new State:
 * 	- Add a new State to the LWRStatus enum in the LWRStatemachine class
 * 	- Generate a new class derived from this interface (LWRState) and write the desired functions CalcControlParam(LWRStatemachine lwrStatemachine), SetACKPacket(LWRStatemachine lwrStatemachine) and InterpretCMDPacket(LWRStatemachine lwrStatemachine)
 * 	- Add this State to the if-else block in the LWRStatemachine function CheckTransistionRequest() and define the transition condition
 * 	- Send the Command plus parameters and check if it works
 * </pre>
 * 
 * @see LwrIdle
 * @see LwrGravComp
 * @see LwrVirtualFixtures
 * @see LwrPathImp
 * @see LwrMoveToPose
 *
 */
public interface ILwrState {
    

    //***************************Methods***********************/
    /**
     * In this Function control Mode Parameters are set and the commanded pose
     * are calculated due the current LWR State. For Further information see the
     * Documentation of the particular State.
     * 
     * @param lwrStatemachine
     *            The operated state machine
     *            @param currentRobotDataSet current robot data.
     * @see LwrIdle
     * @see LwrGravComp
     * @see LwrVirtualFixtures
     * @see LwrPathImp
     * @see LwrMoveToPose
     */
    void calcControlParam(LwrStatemachine lwrStatemachine,
	    RobotDataSet currentRobotDataSet);

    /**
     * In this Function the Command String which is received from the State
     * Control is read and interpreted due to the Current State and if requested
     * and allowed the State is Changed. For Further information see the
     * Documentation of the particular State.
     * 
     * @param lwrStatemachine
     *            - The operated state machine
     * @param cmdPacket
     *            the packet to be interpreted.
     * @throws UnsupportedEncodingException
     * @see LwrIdle
     * @see LwrGravComp
     * @see LwrVirtualFixtures
     * @see LwrPathImp
     * @see LwrMoveToPose
     */
    void interpretCmdPacket(LwrStatemachine lwrStatemachine,
	    CommandPacket cmdPacket);

    /**
     * In this Function the Acknowledge String which is send to the State
     * Control is defined due the current LWR State. For Further information see
     * the Documentation of the particular State.
     * 
     * @param lwrStatemachine
     *            The operated state machine
     * @see LwrIdle
     * @see LwrGravComp
     * @see LwrVirtualFixtures
     * @see LwrPathImp
     * @see LwrMoveToPose
     */
    void setAckPacket(LwrStatemachine lwrStatemachine);
    
    
    /**
     * Connects an external logger to the states.
     * @param extlogger the external logger.
     */
    void setLogger(ITaskLogger extlogger);
}
