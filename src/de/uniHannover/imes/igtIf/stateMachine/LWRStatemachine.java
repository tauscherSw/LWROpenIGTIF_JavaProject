package de.uniHannover.imes.igtIf.stateMachine;

import java.io.UnsupportedEncodingException;

import org.medcare.igtl.messages.OpenIGTMessage;
import org.medcare.igtl.messages.StringMessage;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;

/**
 * State machine class using the LWRState interface and its sub class/states.
 * This class owns a object of this LWRState interface class which changed its
 * class according to the CMDIGTMessage. Currently implemented states are : -
 * LWRIdle - LWRGravComp - LWRError - LWRRegistration - LWRSendData -
 * LWRWaitforData - LWRVirtualFixtures - LWRMoveToPose - LWRPathImp An example
 * use of a state machine application see the imesStateApplication. @ see
 * LWRState @ see imesStateApplication
 * 
 * @see LWRIdle
 * @see LWRGravComp
 * @see LWRError
 * @see LWRRegistration
 * @see LWRSendData
 * @see LWRWaitforData
 * @see LWRVirtualFixtures
 * @see LWRMoveToPose
 * @see LWRPathImp
 * @author Sebastian Tauscher
 *
 */
public class LWRStatemachine {
    /**
     * Number of currently saved Registrations Points
     */
    public int RegistrationPoints = 0;

    /**
     * Current enum for the State Machine status {IDLE, GravComp,
     * VirtualFixtures, PathImp, MovetoPose}possible client states
     */
    public static enum LWRStatus {
	IDLE, GravComp, VirtualFixtures, PathImp, MovetoPose
    }; // possible client states

    /**
     * current status of the client status
     */
    public LWRStatus RobotState = LWRStatus.IDLE; // start as stopped status
    /**
     * ACknowledgement OpenIGTLink Message for the state machine Interface.
     */
    public OpenIGTMessage AckIGTmessage = null;

    /**
     * Command OpenIGTLink Message for the state machine interface
     */
    public OpenIGTMessage CmdIGTmessage = null;
    /**
     * Error Message which is attached to the OpenIGT Status Message in cas e of
     * an error.
     */
    public String ErrorMessage = "No ErrorMessage";
    /**
     * Vector array to save the registration points
     */
    public Vector[] RegPoints = null;

    /**
     * Current State machine UID
     */
    public int UID = 0;
    /**
     * The current State of the LWR state machine
     */
    public ILWRState m_CurrentState;

    /**
     * The new State of the LWR state machine
     */
    public ILWRState m_newState;

    /**
     * The current pose in Cartesian space of the LWR in robot coordinates.
     */
    public MatrixTransformation curPose;

    /**
     * The command pose in Cartesian space of the LWR in robot coordinates.
     */
    public MatrixTransformation cmdPose;

    /**
     * The Transformation from the robot coordinate system or to the images
     * space coordinate system.
     */
    public MatrixTransformation TransformRobotImage;
    /**
     * Flag to identify if the Transform from image to robot space was
     * successfully .
     */
    public boolean TransformRecieved = false;

    /**
     * Flag to identify if the LWR State was changed in the current cycle.
     */
    public boolean ErrorFlag = false;

    /**
     * Flag to identify if the LWR State was changed in the current cycle.
     */
    public boolean InitFlag = true;

    /**
     * Flag to identify if the Registration points were saved successfully .
     */
    public boolean RegistrationFinished = false;

    /**
     * Flag to identify if the Registration points were send successfully to the
     * State Control.
     */
    public boolean SendData = false;
    /**
     * The control mode of the operated state machine
     */
    public IMotionControlMode controlMode;

    // 1 =IMAGESPACE, 2= ROBOTERBASE,3= JOINTSPACE }; //possible Visual
    // interface datatypes
    public int currentVisualIFDatatype = 2; // start with Imagespace
    /**
     * Flag to identify if the Visualization should be started or not.
     */
    public boolean StartVisual = false;

    /**
     * Flag to identify if the StateMachine should be shutdown.
     */
    public boolean Shutdown = false;
    /**
     * Flag to identify if the StateMachine should be Stopped.The robot state is
     * set to IDLE and the robot is holding the current position
     */
    public boolean End = false;
    /**
     * Flag to identify if the StateMachine should be Stopped in the
     * fastest.Attention the programm will stop without changing state.
     */
    public boolean Quit = false;

    /**
     * The Error code for the Error handling. There for the Status code of the
     * OpneIGTLink Protocol are used:<br>
     * 00 - Invalid packet<br>
     * 01 - OK (Default status)<br>
     * 02 - Unknown error <br>
     * 03 - Panic mode<br>
     * 04 - Not found<br>
     * 05 - Acces denied 06 - Busy<br>
     * 07 - Time out / Connection lost <br>
     * 08 - Overflow / Can't be reached<br>
     * 09 - Checksum error <br>
     * 10 - Configuration error <br>
     * 11 - Not enough resource (memory, storage etc)<br>
     * 12 - Illegal/Unknown instruction (or feature not implemented / Unknown
     * command received)<br>
     * 13 - Device not ready (starting up)<br>
     * 14 - Manual mode (device does not accept commands)<br>
     * 15 - Device disabled<br>
     * 16 - Device not present<br>
     * 17 - Device version not known<br>
     * 18 - Hardware failure<br>
     * 19 -Exiting / shut down in progress<br>
     * 
     * @see http://openigtlink.org/protocols/v2_status.html
     */
    public int ErrorCode = 1;
    private String LastPrintedError = "";
    public JointPosition curJntPose;

    /**
     * Constructor of LWRStatemachine. The Current state is set to the save
     * state Idle
     */
    public LWRStatemachine() {
	m_CurrentState = new LWRIdle();
	AckIGTmessage = new StringMessage("ACK_0", "IDLE;");
	CmdIGTmessage = new StringMessage("CMD_0", "IDLE;");
	SendData = false;
	InitFlag = true;
	TransformRecieved = false;
	ErrorCode = 1;

    }

    /**
     * This function change the current state of the robot to the new state.
     * 
     * @param the
     *            new LWRState
     */
    public void ChangeLWRState(ILWRState newState) {
	m_CurrentState = newState;
    }

    /**
     * calls the CalcControlParam of the current LWR state.
     * 
     * @see ILWRState
     * @see LWRIdle
     * @see LWRGravComp
     * @see LWRRegistration
     * @see LWRSendData
     * @see LWRWaitforData
     * @see LWRVirtualFixtures
     * @see LWRPathImp
     * @see LWRMovetoPose
     */
    public void CalcControlParam() {
	m_CurrentState.CalcControlParam(this);
    }

    /**
     * calls the SetACKPacket of the current LWR state.
     * 
     * @see ILWRState
     * @see LWRIdle
     * @see LWRGravComp
     * @see LWRRegistration
     * @see LWRSendData
     * @see LWRWaitforData
     * @see LWRVirtualFixtures
     * @see LWRPathImp
     * @see LWRMovetoPose
     */
    public void SetACKPacket() {
	m_CurrentState.SetACKPacket(this);
    }

    /**
     * Calls the InterpretCMDPacket of the current LWR state.
     * 
     * @see ILWRState
     * @see LWRIdle
     * @see LWRGravComp
     * @see LWRRegistration
     * @see LWRSendData
     * @see LWRWaitforData
     * @see LWRVirtualFixtures
     * @see LWRPathImp
     * @see LWRMovetoPose
     */
    public void InterpretCMDPacket() {
	m_CurrentState.InterpretCMDPacket(this);
    }

    /**
     * This functions checks if a Transition Request was received and then
     * checks if this Transition is allowed. When a new State is added then an
     * else-if block needs to be added and the transition condition needs to be
     * checked
     */
    public void CheckTransitionRequest() {
	// First Check if the received OpenIGTLink was a String
	if (this.CmdIGTmessage.getHeader().getDataType().equals("STRING")) {
	    String CMD_String;
	    try {
		CMD_String = new String(this.CmdIGTmessage.getBody(), 4,
			this.CmdIGTmessage.getBody().length - 4, "US-ASCII");
		String[] CMD_Array = CMD_String.split(";");
		// Check if the recieved State is the current State
		if (RobotState.name().equalsIgnoreCase(CMD_Array[0])) {

		} else { // If not check if the Transition is allowed
			 // Set the init flag true
		    this.InitFlag = true;

		    // Check if the Transition Request is allowed and change the
		    // State if it is allowed
		    if (CMD_Array[0].contentEquals("IDLE")) {
			// ToDO: Add check if it is allowed e.g. if(flagX &&
			// flagY &&flagZ || ...) what ever you want
			LWRIdle newState = new LWRIdle();
			this.ChangeLWRState(newState);
			RobotState = LWRStatus.IDLE;
			this.ErrorCode = 1;

		    } else if (CMD_Array[0].contentEquals("GravComp")) {
			if (RobotState == LWRStatus.IDLE
				|| RobotState == LWRStatus.VirtualFixtures
				|| RobotState == LWRStatus.PathImp) {
			    // ToDO: Add check if it is allowed e.g. if(flagX &&
			    // flagY &&flagZ || ...) what ever you want
			    LWRGravComp newState = new LWRGravComp();
			    this.ChangeLWRState(newState);
			    RobotState = LWRStatus.GravComp;
			    this.ErrorCode = 1;
			} else {
			    this.ErrorMessage = ("Transition from State "
				    + RobotState.toString() + " to "
				    + CMD_Array[0] + " is not allowed!");
			    ErrorFlag = true;
			    this.ErrorCode = 10;
			}
		    } else if (CMD_Array[0].contentEquals("VirtualFixtures")) {
			// ToDO: Add check if it is allowed e.g. if(flagX &&
			// flagY &&flagZ || ...) what ever you want
			if (RobotState == LWRStatus.IDLE
				|| RobotState == LWRStatus.GravComp
				|| RobotState == LWRStatus.PathImp) {
			    if (CMD_Array.length >= 9) {
				LWRVirtualFixtures newState = new LWRVirtualFixtures();

				this.ChangeLWRState(newState);
				this.ErrorCode = 1;
				RobotState = LWRStatus.VirtualFixtures;
			    } else {
				this.ErrorMessage = ("Not enough Parameters recieved for the VirtualFixture State (recieved : "
					+ CMD_Array.length + ", expected : 9)");
				ErrorFlag = true;
				this.ErrorCode = 10;
			    }
			} else {
			    this.ErrorMessage = ("Transition from State "
				    + RobotState.toString() + " to "
				    + CMD_Array[0] + " is not allowed!");
			    ErrorFlag = true;
			    this.ErrorCode = 10;
			}
		    } else if (CMD_Array[0].contentEquals("PathImp")) {
			// ToDO: Add check if it is allowed e.g. if(flagX &&
			// flagY &&flagZ || ...) what ever you want
			if (RobotState == LWRStatus.IDLE
				|| RobotState == LWRStatus.GravComp
				|| RobotState == LWRStatus.VirtualFixtures) {
			    if (CMD_Array.length == 5) {
				LWRPathImp newState = new LWRPathImp();
				this.ChangeLWRState(newState);
				this.ErrorCode = 1;
				RobotState = LWRStatus.PathImp;
			    } else {
				this.ErrorMessage = ("Unexpected number of parameters recieved for the PathImp State (recieved : "
					+ CMD_Array.length + ", expected : 5)");
				this.ErrorFlag = true;
				this.ErrorCode = 10;
			    }

			} else {
			    this.ErrorMessage = ("Transition from State "
				    + RobotState.toString() + " to "
				    + CMD_Array[0] + " is not allowed!");
			    ErrorFlag = true;
			    this.ErrorCode = 10;
			}
		    } else if (CMD_Array[0].contentEquals("MoveToPose")) {
			// ToDO: Add check if it is allowed e.g. if(flagX &&
			// flagY &&flagZ || ...) what ever you want
			if (RobotState == LWRStatus.IDLE) {
			    LWRMoveToPose newState = new LWRMoveToPose();
			    if (CMD_Array.length != 8) {
				this.ErrorMessage = ("Unexpected number of parameters recieved for the MovetoPose State (recieved : "
					+ CMD_Array.length + ", expected : 8)");
				this.ErrorFlag = true;
				this.ErrorCode = 10;

			    } else {
				this.ChangeLWRState(newState);
				this.ErrorCode = 1;
				RobotState = LWRStatus.MovetoPose;
			    }
			} else {
			    this.ErrorMessage = ("Transition from State "
				    + RobotState.toString() + " to "
				    + CMD_Array[0] + " is not allowed!");
			    ErrorFlag = true;
			    this.ErrorCode = 10;
			}
		    } else if (CMD_Array[0].contentEquals("Visual")) {
			// ToDO: Add check if it is allowed e.g. if(flagX &&
			// flagY &&flagZ || ...) what ever you want
			if (CMD_Array.length != 3) {
			    this.ErrorMessage = ("Unexpected number of parameters recieved to Start the Visual interface (recieved : "
				    + CMD_Array.length + ", expected : 5)");
			    this.ErrorFlag = true;
			    this.ErrorCode = 10;
			    this.InitFlag = false;
			} else {
			    if (CMD_Array[1].contentEquals("true")) {
				this.StartVisual = true;
				if (CMD_Array[2].contentEquals("jnt")) {
				    this.currentVisualIFDatatype = 3;
				} else if (CMD_Array[1].contentEquals("rob")) {
				    this.currentVisualIFDatatype = 2;
				} else if (CMD_Array[2].contentEquals("img")) {
				    this.currentVisualIFDatatype = 1;
				}
				System.out
					.println("StateMachine: Visual IF started with  datatype "
						+ this.currentVisualIFDatatype
						+ "(1=img, 2=rob, 3=jnt)");
			    } else if (CMD_Array[1].contentEquals("false")) {
				this.StartVisual = false;
			    }
			    this.ErrorCode = 1;
			    this.InitFlag = false;

			}
			// Now checking if the command to stop the State machine
			// (Quit, End, Shutdown) was received
		    } else if (CMD_Array[0].contentEquals("Quit")) {
			this.Quit = true;
			this.InitFlag = false;
		    } else if (CMD_Array[0].contentEquals("End")) {
			this.End = true;
			this.InitFlag = true;
			this.ErrorCode = 1;
			LWRIdle newState = new LWRIdle();
			this.ChangeLWRState(newState);
			RobotState = LWRStatus.IDLE;

		    } else if (CMD_Array[0].contentEquals("Shutdown")) {
			this.End = true;
			this.InitFlag = true;
			this.ErrorCode = 1;
			LWRIdle newState = new LWRIdle();
			this.ChangeLWRState(newState);
			RobotState = LWRStatus.IDLE;

		    } else {
			this.ErrorCode = 12; // Illegal/Unknown instruction
			this.ChangeLWRState(new LWRError());
			this.ErrorMessage = "Unexpected COMMAND recieved! See the list opf supported Commands (received: "
				+ CMD_Array[0] + ")";
		    }

		    // If the State was Changed then interpret the CMD Packet
		    // according to the Current stat
		    if (this.InitFlag && !this.ErrorFlag) {
			this.m_CurrentState.InterpretCMDPacket(this);
		    }

		}

	    } catch (UnsupportedEncodingException e) {
		// TODO Automatisch generierter Erfassungsblock
		e.printStackTrace();
	    }
	} else {
	    this.InitFlag = false;
	}

    }

    public void ErrorHandler() {
	if (this.ErrorFlag && !this.LastPrintedError.equals(ErrorMessage)) {

	    // Print the new ErrorMessage
	    System.out.println(this.ErrorMessage);
	    // If necessary change robot state to LWRError
	    if (ErrorCode == 18) {
		LWRError newState = new LWRError();
		ChangeLWRState(newState);
	    }
	    this.LastPrintedError = ErrorMessage;
	    this.ErrorFlag = false;
	} else {
	    this.LastPrintedError = "";
	}
    }

}
