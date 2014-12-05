package de.uniHannover.imes.igtIf.stateMachine;

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
 * @author Sebastian Tauscher
 * @see LWRIdle
 * @see LWRGravComp
 * @see LWRVirtualFixtures
 * @see LWRPathImp
 * @see LWRMovetoPos
 *
 */
interface LWRState {
    /**
     * In this Function control Mode Parameters are set and the commanded pose
     * are calculated due the current LWR State. For Further information see the
     * Documentation of the particular State.
     * 
     * @param lwrStatemachine
     *            The operated state machine
     * @see LWRIdle
     * @see LWRGravComp
     * @see LWRRegistration
     * @see LWRSendData
     * @see LWRWaitforData
     * @see LWRVirtualFixtures
     * @see LWRPathImp
     * @see LWRMovetoPose
     */

    public static enum VisualIFDatatype {
	IMAGESPACE, ROBOTERBASE, JOINTSPACE
    }; // possible Visual interface datatypes

    void CalcControlParam(LWRStatemachine lwrStatemachine);

    /**
     * In this Function the Acknowledge String which is send to the State
     * Control is defined due the current LWR State. For Further information see
     * the Documentation of the particular State.
     * 
     * @param lwrStatemachine
     *            The operated state machine
     * @see LWRIdle
     * @see LWRGravComp
     * @see LWRRegistration
     * @see LWRSendData
     * @see LWRWaitforData
     * @see LWRVirtualFixtures
     * @see LWRPathImp
     * @see LWRMovetoPose
     */
    void SetACKPacket(LWRStatemachine lwrStatemachine);

    /**
     * In this Function the Command String which is received from the State
     * Control is read and interpreted due to the Current State and if requested
     * and allowed the State is Changed. For Further information see the
     * Documentation of the particular State.
     * 
     * @param lwrStatemachine
     *            - The operated state machine
     * @throws UnsupportedEncodingException
     * @see LWRIdle
     * @see LWRGravComp
     * @see LWRRegistration
     * @see LWRSendData
     * @see LWRWaitforData
     * @see LWRVirtualFixtures
     * @see LWRPathImp
     * @see LWRMovetoPose
     */
    void InterpretCMDPacket(LWRStatemachine lwrStatemachine);
}
