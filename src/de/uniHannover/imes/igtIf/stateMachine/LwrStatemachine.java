/*=========================================================================

  Program:   LWRStatemachine
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

 Copyright (c) Sebastian Tauscher, Institute of Mechatronics Systems, Leibniz Universit�t Hannover. All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
	
 * Redistributions of source code must retain the above copyright notice,
	    this list of conditions and the following disclaimer.
	
 * Redistributions in binary form must reproduce the above copyright notice,
	    this list of conditions and the following disclaimer in the documentation 
	    and/or other materials provided with the distribution.
	
 * Neither the name of the Insight Software Consortium nor the names of its 
	    contributors may be used to endorse or promote products derived from this 
	    software without specific prior written permission.
	
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

package de.uniHannover.imes.igtIf.stateMachine;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;

import de.uniHannover.imes.igtIf.communicationIf.LWRVisualizationInterface
.VisualIFDatatypes;

/**
 * State machine class using the LWRState interface and its sub class/states.
 * This class owns a object of this LWRState interface class which changed its
 * class according to the CMDIGTMessage. Currently implemented states are : -
 * LWRIdle - LWRGravComp - LWRVirtualFixtures - LWRMoveToPose - LWRPathImp An
 * example use of a state machine application see the imesStateApplication.
 * 
 * @see ILwrState
 * @see LwrIdle
 * @see LwrGravComp
 * @see LwrVirtualFixtures
 * @see LwrMoveToPose
 * @see LwrPathImp
 * @author Sebastian Tauscher
 */
public class LwrStatemachine {

    /**
     * Current enum for the State Machine status {IDLE, GravComp,
     * VirtualFixtures, PathImp, MovetoPose, Error}possible client states.
     */
    public static enum LWRStatus {
	IDLE, GravComp, VirtualFixtures, PathImp, MovetoPose
    }

    /**
     * The Error code for the Error handling. There for the Status code of the
     * OpneIGTLink Protocol are used:<br>
     * {@linkplain} http://openigtlink.org/protocols/v2_status.html}
     * 
     */
    public static enum OpenIGTLinkErrorCode {
	/** Invalid packet. */
	InvalidPacket(0),
	/** Ok (Default status. */
	Ok(1),
	/** Unknown error. */
	UnknownError(2),
	/** Panic mode. */
	PanicMode(3),
	/** Not found. */
	NotFound(4),
	/** Acces denied 06 - Busy. */
	AccessDenied(5),
	/** Time out / Connection lost. */
	TimeOut(7),
	/** Overflow / Can't be reached.*/
	Overflow(8),
	/** Checksum error. */
	ChecksumError(9),
	/** Configuration error. */
	ConfigurationError(10),
	/** Not enough resource (memory, storage etc). */
	NotEnoughResource(11),
	/**
	 * Illegal/Unknown instruction (or feature not implemented / Unknown
	 * command received).
	 */
	IllegalInstruction(12),
	/**
	 * Illegal/Unknown instruction (or feature not implemented / Unknown
	 * command received).
	 */
	UnknownInstruction(12),
	/** Device not ready (starting up). */
	DeviceNotReady(13),
	/** Manual mode (device does not accept commands). */
	ManualMode(14),
	/** Device disabled. */
	DeviceDisabled(15),
	/** Device not present. */
	DeviceNotPresent(16),
	/** Device version not known. */
	DeviceVersionNotFound(17),
	/** Hardware failure. */
	HarwareFailure(18),
	/** Exiting / shut down in progress. */
	Exiting(19);

	/** error number corresponding to the openIGTLink defined error numbers. */
	private final int errorNumber;

	/** maximum error number. */
	private static final int MAX_ERROR_NUM = 20;

	/** minimum error number. */
	private static final int MIN_ERROR_NUM = 0;

	/**
	 * Constructs the different error-code-objects and validates the number.
	 * 
	 * @param errorNum
	 *            the number of the error according to the openIGTLink
	 *            protocol.
	 */
	private OpenIGTLinkErrorCode(final int errorNum) {
	    if (errorNum >= MIN_ERROR_NUM && errorNum <= MAX_ERROR_NUM) {
		this.errorNumber = errorNum;
	    } else {
		throw new IllegalArgumentException("only error codes between "
			+ MIN_ERROR_NUM + " and " + MAX_ERROR_NUM
			+ " are supported.");
	    }
	}
	
	/** Method to access the error numbers.
	 * 
	 * @return the error number of the current error code.
	 */
	public int getErrorNumber() {
	
	    return (new Integer(errorNumber));
	}
    }

    /**
     * Represents the current datatype of the visualization interface.
     */
    public VisualIFDatatypes currentVisualIFDatatype = 
	    VisualIFDatatypes.ROBOTBASE;

    /**
     * visualInterfaceDatatype.Robotbase current status of the client status.
     */
    public LWRStatus RobotState = LWRStatus.IDLE; // start as stopped status

    /**
     * ACknowledgement OpenIGTLink Message for the state machine Interface.
     */
    public String AckIGTmessage = null;

    /**
     * Command OpenIGTLink Message for the state machine interface.
     */
    public String CmdIGTmessage = null;
    /**
     * Error Message which is attached to the OpenIGT Status Message in cas e of
     * an error.
     */
    public String ErrorMessage = "";

    /**
     * Current State machine UID.
     */
    public long UID = 0;
    /**
     * The current State of the LWR state machine.
     */
    public ILwrState m_CurrentState;

    /**
     * The new State of the LWR state machine.
     */
    public ILwrState m_newState;

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
     * The control mode of the operated state machine.
     */
    public IMotionControlMode controlMode;

    /**
     * Flag to identify if the Visualization should be started or not.
     */
    public boolean StartVisual = false;
    /**
     * String containing the data type of the IGTLink Message which is received.
     */
    public String IGTLdatatype = "STRING";

    /**
     * Flag to identify if the StateMachine should be Stopped.The robot state is
     * set to IDLE and the robot is holding the current position.
     */
    public boolean End = false;

    /** the current ErrorCode. */
    public OpenIGTLinkErrorCode ErrorCode = OpenIGTLinkErrorCode.Ok;

    /**
     * String containing the last printed Error Message. This String is used to
     * avoid to print the same error message again and again.
     */
    private String LastPrintedError = "";

    /**
     * Current Joint Position of the LWR received via SmartServo.
     */
    public JointPosition curJntPose;
    /**
     * Current Stiffness of the LWR as a 1x6 stiffness vector (x, y, z, A, B, C).
     */
    public int[] curCartStiffness = { 0, 0, 0, 0, 0, 0 };
    /**
     * SubString containing the Parameters set of the received Command String,
     * e.g. the VirtualFixtures definition or the destination point for
     * MoveTo/Path.
     */
    public String ParameterString = "";

    /**
     * UID of the Pose gotten via SmartServo. The value is increased each time
     * new data is read from the robot.
     */
    public int PoseUID = 0;

    /**
     * Vector containing the force estimated at the tool center point by the
     * internal torque sensors.
     */
    public Vector TCPForce;

    /**
     * Constructor of LWRStatemachine. The Current state is set to the save
     * state Idle
     */
    public LwrStatemachine() {
	m_CurrentState = new LwrIdle();
	AckIGTmessage = "IDLE;";
	CmdIGTmessage = "IDLE;";
	InitFlag = true;
	TransformRecieved = false;
	ErrorCode = OpenIGTLinkErrorCode.Ok;

    }

    /**
     * This function change the current state of the robot to the new state.
     * 
     * @param newState
     *            the new LWRState
     */
    public final void changeLWRState(ILwrState newState) {
	m_CurrentState = newState;
    }

    /**
     * calls the CalcControlParam of the current LWR state.
     * 
     * @see ILwrState
     * @see LwrIdle
     * @see LwrGravComp
     * @see LwrVirtualFixtures
     * @see LwrPathImp
     * @see LwrMoveToPose
     */
    public final void calcControlParam() {
	m_CurrentState.calcControlParam(this);
    }

    /**
     * calls the SetACKPacket of the current LWR state.
     * 
     * @see ILwrState
     * @see LwrIdle
     * @see LwrGravComp
     * @see LwrVirtualFixtures
     * @see LwrPathImp
     * @see LwrMoveToPose
     */
    public final void setAckPacket() {
	m_CurrentState.setAckPacket(this);
    }

    /**
     * Calls the InterpretCMDPacket of the current LWR state.
     * 
     * @see ILwrState
     * @see LwrIdle
     * @see LwrGravComp
     * @see LwrVirtualFixtures
     * @see LwrPathImp
     * @see LwrMoveToPose
     */
    public final void interpretCmdPacket() {
	m_CurrentState.interpretCmdPacket(this);
    }

    /**
     * This functions checks if a Transition Request was received and then
     * checks if this Transition is allowed. When a new State is added then an
     * else-if block needs to be added and the transition condition needs to be
     * checked
     * 
     * <pre>
     *   <code>
     *  {@code	
     *  To ADD a state use the template below:
     * 	else if (CMD_Array[0].contentEquals("NameofState")){
     * 		ToDO: Add check if it is allowed e.g. if(flagX && flagY &&flagZ || ...) what ever you want
     * 		if(flagX && flagY &&flagZ || OldState == XX) {
     * 			LWRNameofState newState = new LWRNameofState();
     * 			if (CMD_Array.length != 8 ){
     * 				this.ErrorMessage = ("Unexpected number of parameters received for the NameofState State (received : " +CMD_Array.length + ", expected : 8)");
     * 				this.ErrorFlag = true;
     * 				this.ErrorCode = 10;
     * 					
     * 			}else{
     * 				this.ChangeLWRState(newState);
     * 				//Set the init flag true
     * 				this.InitFlag = true;
     * 				this.ErrorFlag = false;
     * 				this.ErrorCode = 1;
     * 				RobotState = LWRStatus.NameofStatee;
     * 			}
     * 		}else{
     * 			this.ErrorMessage = ("Transition to State NameofState is not allowed!");		
     * 			ErrorFlag = true;
     * 			this.ErrorCode = 10;
     * 		}
     * 	}
     * }
     *  </code>
     * </pre>
     */
    public void CheckTransitionRequest() {
	// First Check if the received OpenIGTLink was a String
	if (this.IGTLdatatype.equals("STRING")) {
	    // this.InitFlag = false;
	    String CMD_String;
	    CMD_String = CmdIGTmessage;
	    // Split String into a String array with ";" as a separator
	    String[] CMD_Array = CMD_String.split(";");
	    // Check if the recieved State is the current State
	    if (RobotState.name().equalsIgnoreCase(CMD_Array[0])) {
		// Get new parameter String from CommandString
		if (RobotState == LWRStatus.MovetoPose
			|| RobotState == LWRStatus.VirtualFixtures
			|| RobotState == LWRStatus.PathImp) {
		    String NewParameterString = CMD_String.substring(CMD_String
			    .indexOf(";"));
		    // If paramter Set changed then re-init the state
		    if (!ParameterString.equals(NewParameterString)) {
			this.InitFlag = true;
		    }
		}
	    } else { // If not check if the Transition is allowed

		// Check if the Transition Request is allowed and change the
		// State if it is allowed
		if (CMD_Array[0].contentEquals("IDLE")) { // Transition Request
							  // equal "IDLE"
		    // ToDO: Add check if it is allowed e.g. if(flagX && flagY
		    // &&flagZ || ...) what ever you want
		    LwrIdle newState = new LwrIdle();
		    this.InitFlag = true;
		    this.changeLWRState(newState);
		    this.ErrorFlag = false;
		    RobotState = LWRStatus.IDLE;
		    this.ErrorCode = OpenIGTLinkErrorCode.Ok;

		} else if (CMD_Array[0].contentEquals("GravComp")) { // Transistion
								     // Request
								     // equal
								     // "GravComp"
		    if (RobotState == LWRStatus.IDLE
			    || RobotState == LWRStatus.VirtualFixtures
			    || RobotState == LWRStatus.PathImp) {
			// ToDO: Add check if it is allowed e.g. if(flagX &&
			// flagY &&flagZ || ...) what ever you want
			LwrGravComp newState = new LwrGravComp();
			this.changeLWRState(newState);
			this.ErrorFlag = false;
			// Set the init flag true
			this.InitFlag = true;
			RobotState = LWRStatus.GravComp;
			this.ErrorCode = OpenIGTLinkErrorCode.Ok;
		    } else {
			this.ErrorMessage = ("Transition to GravComp is not allowed!");
			ErrorFlag = true;
			this.ErrorCode = OpenIGTLinkErrorCode.ConfigurationError;
		    }
		} else if (CMD_Array[0].contentEquals("VirtualFixtures")) {// Transistion
									   // Request
									   // equal
									   // "VirtualFixtures"
		    // ToDO: Add check if it is allowed e.g. if(flagX && flagY
		    // &&flagZ || ...) what ever you want
		    if (RobotState == LWRStatus.IDLE
			    || RobotState == LWRStatus.GravComp
			    || RobotState == LWRStatus.PathImp) {
			if (CMD_Array.length >= 9) {
			    LwrVirtualFixtures newState = new LwrVirtualFixtures();

			    this.changeLWRState(newState);
			    // Set the init flag true
			    this.InitFlag = true;
			    this.ErrorCode = OpenIGTLinkErrorCode.Ok;
			    RobotState = LWRStatus.VirtualFixtures;

			    // Check if the correct numbers of parameters was
			    // received
			} else {
			    this.ErrorMessage = ("Not enough Parameters recieved for the VirtualFixture State (recieved : "
				    + CMD_Array.length + ", expected : 9)");
			    ErrorFlag = true;
			    this.ErrorCode = OpenIGTLinkErrorCode.ConfigurationError;
			}
		    } else {
			this.ErrorMessage = ("Transition to State VirtualFixtures is not allowed!");
			ErrorFlag = true;
			this.ErrorCode = OpenIGTLinkErrorCode.ConfigurationError;
		    }
		} else if (CMD_Array[0].contentEquals("PathImp")) {
		    // ToDO: Add check if it is allowed e.g. if(flagX && flagY
		    // &&flagZ || ...) what ever you want
		    if (RobotState == LWRStatus.IDLE
			    || RobotState == LWRStatus.GravComp
			    || RobotState == LWRStatus.VirtualFixtures) {
			if (CMD_Array.length == 5) {
			    LwrPathImp newState = new LwrPathImp();
			    this.changeLWRState(newState);
			    // Set the init flag true
			    this.InitFlag = true;
			    this.ErrorFlag = false;
			    this.ErrorCode = OpenIGTLinkErrorCode.Ok;
			    RobotState = LWRStatus.PathImp;

			    // Check if the correct numbers of parameters was
			    // received
			} else {
			    this.ErrorMessage = ("Unexpected number of parameters recieved for the PathImp State (recieved : "
				    + CMD_Array.length + ", expected : 5)");
			    this.ErrorFlag = true;
			    this.ErrorCode = OpenIGTLinkErrorCode.ConfigurationError;
			}

		    } else {
			this.ErrorMessage = ("Transition to State PathImp is not allowed!");
			ErrorFlag = true;
			this.ErrorCode = OpenIGTLinkErrorCode.ConfigurationError;
		    }
		} else if (CMD_Array[0].contentEquals("MoveToPose")) {
		    // ToDO: Add check if it is allowed e.g. if(flagX && flagY
		    // &&flagZ || ...) what ever you want
		    if (RobotState == LWRStatus.IDLE && this.TransformRecieved) {
			LwrMoveToPose newState = new LwrMoveToPose();
			if (CMD_Array.length != 8) {
			    this.ErrorMessage = ("Unexpected number of parameters recieved for the MovetoPose State (recieved : "
				    + CMD_Array.length + ", expected : 8)");
			    this.ErrorFlag = true;
			    this.ErrorCode = OpenIGTLinkErrorCode.ConfigurationError;

			} else {
			    this.changeLWRState(newState);
			    // Set the init flag true
			    this.InitFlag = true;
			    this.ErrorFlag = false;
			    this.ErrorCode = OpenIGTLinkErrorCode.Ok;
			    RobotState = LWRStatus.MovetoPose;
			}
		    } else {
			this.ErrorMessage = ("Transition to State MoveToPose is not allowed!");
			ErrorFlag = true;
			this.ErrorCode = OpenIGTLinkErrorCode.IllegalInstruction;
		    }
		    // To ADD a state use the template below:
		    // else if (CMD_Array[0].contentEquals("NameofState")){
		    // ToDO: Add check if it is allowed e.g. if(flagX && flagY
		    // &&flagZ || ...) what ever you want
		    // if(flagX && flagY &&flagZ || OldState == XX) {
		    // LWRNameofState newState = new LWRNameofState();
		    // if (CMD_Array.length != 8 ){
		    // this.ErrorMessage =
		    // ("Unexpected number of parameters received for the NameofState State (received : "
		    // +CMD_Array.length + ", expected : 8)");
		    // this.ErrorFlag = true;
		    // this.ErrorCode = 10;
		    //
		    // }else{
		    // this.ChangeLWRState(newState);
		    // //Set the init flag true
		    // this.InitFlag = true;
		    // this.ErrorFlag = false;
		    // this.ErrorCode = 1;
		    // RobotState = LWRStatus.NameofStatee;
		    // }
		    // }else{
		    // this.ErrorMessage =
		    // ("Transition to State NameofState is not allowed!");
		    // ErrorFlag = true;
		    // this.ErrorCode = 10;
		    // }
		    // From here one here are additional commands to operate the
		    // state machine which are not transition requests
		} else if (CMD_Array[0].contentEquals("Visual")) {
		    // ToDO: Add check if it is allowed e.g. if(flagX && flagY
		    // &&flagZ || ...) what ever you want
		    if (CMD_Array.length != 3) {
			this.ErrorMessage = ("Unexpected number of parameters recieved to Start the Visual interface (recieved : "
				+ CMD_Array.length + ", expected : 5)");
			this.ErrorFlag = true;
			this.ErrorCode = OpenIGTLinkErrorCode.ConfigurationError;
			this.InitFlag = false;
		    } else {
			if (CMD_Array[1].contentEquals("true")
				&& this.StartVisual == false) {
			    this.StartVisual = true;
			    if (CMD_Array[2].contentEquals("jnt")) {
				this.currentVisualIFDatatype = VisualIFDatatypes.JOINTSPACE;
			    } else if (CMD_Array[1].contentEquals("rob")) {
				this.currentVisualIFDatatype = VisualIFDatatypes.ROBOTBASE;
			    } else if (CMD_Array[2].contentEquals("img")) {
				this.currentVisualIFDatatype = VisualIFDatatypes.IMAGESPACE;
			    }
			    System.out
				    .println("StateMachine: Visual IF started with  datatype "
					    + this.currentVisualIFDatatype
					    + "(1=img, 2=rob, 3=jnt)");
			} else if (CMD_Array[1].contentEquals("false")
				&& this.StartVisual == true) {
			    this.StartVisual = false;
			    System.out
				    .println("StateMachine: Visual IF stopped!");
			}
			this.ErrorCode = OpenIGTLinkErrorCode.Ok;
			this.InitFlag = false;

		    }
		    // Now checking if the command to stop the State machine
		    // (Quit, End, Shutdown) was received
		} else if (CMD_Array[0].contentEquals("Shutdown")
			|| CMD_Array[0].contentEquals("End")
			|| CMD_Array[0].contentEquals("Quit")) {
		    this.End = true;
		    this.InitFlag = true;
		    this.ErrorCode = OpenIGTLinkErrorCode.Ok;
		    LwrIdle newState = new LwrIdle();
		    this.changeLWRState(newState);
		    RobotState = LWRStatus.IDLE;

		} else {
		    this.ErrorCode = OpenIGTLinkErrorCode.IllegalInstruction; 
		    this.ErrorMessage = "Unexpected COMMAND recieved! See the list of supported Commands (received: "
			    + CMD_Array[0] + ")";
		    this.ErrorFlag = true;

		}
		if (this.ErrorFlag) {
		    this.InitFlag = false;
		}

	    }
	    // If the State was Changed then interpret the CMD Packet according
	    // to the Current stat
	    if (this.InitFlag && !this.ErrorFlag) {
		this.m_CurrentState.interpretCmdPacket(this);
	    }
	} else {
	    this.InitFlag = false;
	}

    }

    public void ErrorHandler(boolean DebugInfo) {
	if (this.ErrorFlag) {
	    if (!this.LastPrintedError.equals(ErrorMessage)) {
		// Print the new ErrorMessage
		if (DebugInfo) {
		    System.out.println(this.ErrorMessage);
		}
		// If necessary change robot state to LWRError
		if (ErrorCode == OpenIGTLinkErrorCode.HarwareFailure) {
		    if (RobotState == LWRStatus.IDLE
			    || RobotState == LWRStatus.GravComp) {
			LwrIdle newState = new LwrIdle();
			changeLWRState(newState);
			this.InitFlag = true;
			RobotState = LWRStatus.IDLE;
		    } else if (RobotState == LWRStatus.VirtualFixtures
			    || RobotState == LWRStatus.PathImp
			    || RobotState == LWRStatus.MovetoPose) {
			LwrIdle newState = new LwrIdle();
			changeLWRState(newState);
			this.InitFlag = true;
			RobotState = LWRStatus.IDLE;
		    }
		}
		this.LastPrintedError = ErrorMessage;
		this.ErrorFlag = false;
	    }
	} else {
	    this.LastPrintedError = "";
	}
    }

}
