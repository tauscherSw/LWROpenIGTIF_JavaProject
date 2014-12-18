/*=========================================================================

  Program:   SimpleStateExample
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Portions (c) Sebastian Tauscher, Institute of Mechatronics Systems, Leibniz Universit�t Hannover. All rights reserved.

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

package de.uniHannover.imes.igtIf.application;

import java.util.concurrent.TimeUnit;

import de.uniHannover.imes.igtIf.communicationIf.LWRStateMachineInterface;
import de.uniHannover.imes.igtIf.communicationIf.LWRVisualizationInterface;
import de.uniHannover.imes.igtIf.communicationIf.LWRVisualizationInterface.VisualIFDatatypes;
import de.uniHannover.imes.igtIf.stateMachine.LWRStatemachine;
import com.kuka.common.StatisticTimer;
import com.kuka.common.ThreadUtil;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.physics.Inertia;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

public class StateMachineApplication extends RoboticsAPIApplication {

    /** The robot object for controlling robot movements. */
    private LBR imesLBR;

    /** Sunrise specific interface to control the robot's movements. */
    private ISmartServoRuntime smartServoRuntime;
    /**
     * Object of the state machine interface class for the communication with a
     * state control software using the OpenIGTLink protocol.
     * 
     * @see LWRStateMachineInterface
     */
    private LWRStateMachineInterface slicerControlIf;

    /**
     * Object of the visualization interface class for the communication with a
     * visualization software using the OpenIGTLink protocol.
     * 
     * @see LWRVisualizationInterface
     */
    private LWRVisualizationInterface slicerVisualIf;
    /**
     * Object of the State machine class.
     * 
     * @see LWRStatemachine
     */
    private LWRStatemachine imesStatemachine;

    /**
     * Control mode for movements during state control.
     */
    private IMotionControlMode controlMode;
    /**
     * number of loops to run with out any communication with the state control.
     */
    private static final int N_OF_RUNS = 500;

    /**
     * Cyclic time of each loop of the main (state machine) thread.
     */
    private static final int MS_TO_SLEEP = 10;

    /**
     * The tool object describing the physical properties of the tool attached
     * to the robot's flange.
     */
    private final Tool imesTool = new Tool("Imes Tool", new LoadData(0.6,
	    MatrixTransformation.ofTranslation(-5, 0, 50), Inertia.ZERO));

    /**
     * Definition of the tool-center-point.
     */
    private final MatrixTransformation toolTCP = MatrixTransformation
	    .ofTranslation(-40, 10, 207);

    /**
     * Definition of the initial robot position before the state-machine starts
     * working.
     */
    private static final JointPosition INITIAL_ROBOT_POSE = new JointPosition(
	    0.0, Math.toRadians(30), 0., -Math.toRadians(60), 0.,
	    Math.toRadians(90), 0.);

    /**
     * Velocity of robot movements during state-machine execution. Value in %.
     */
    private static final double VEL = 1;

    /**
     * Acceleration of robot movements during state-machine execution. Value in
     * %.
     */
    private static final double ACC = 1;

    /**
     * Minimum trajectory execution time during state-machine execution. Value
     * in seconds.
     */
    private static final double TRAJ_EXC_TIME = 5e-03;

    /**
     * The translational stiffness, when the robot is moving in cartesian
     * impedance mode. Value in N/m.
     */
    private static final double TRANSL_STIFF = 5000;
    /**
     * The rotational stiffness, when the robot is moving in cartesian impedance
     * mode. Value in Nm/rad.
     */
    private static final double ROT_STIFF = 300;
    /**
     * The nullspace stiffness (rotational), when the robot is moving in
     * cartesian impedance mode. Value in Nm/rad.
     */
    private static final double NULLSP_STIFF = 5000;

    /**
     * The maximum path deviation in x direction, when the robot is moving in
     * cartesian impedance mode. Values in mm.
     */
    private static final double PATH_DEV_X = 500;
    /**
     * The maximum path deviation in y direction, when the robot is moving in
     * cartesian impedance mode. Values in mm.
     */
    private static final double PATH_DEV_Y = 500;
    /**
     * The maximum path deviation in z direction, when the robot is moving in
     * cartesian impedance mode. Values in mm.
     */
    private static final double PATH_DEV_Z = 500;
    /**
     * The maximum path-orientation deviation in a-angle, when the robot is
     * moving in cartesian impedance mode. Values in rad.
     */
    private static final double PATH_DEV_A = 10; // TODO check value
    /**
     * The maximum path-orientation deviation in b-angle, when the robot is
     * moving in cartesian impedance mode. Values in rad.
     */
    private static final double PATH_DEV_B = 10; // TODO check value
    /**
     * The maximum path-orientation deviation in c-angle, when the robot is
     * moving in cartesian impedance mode. Values in rad.
     */
    private static final double PATH_DEV_C = 10; // TODO check value

    /**
     * In this function the robot, tool etc are initialized.
     **/
    @Override
    public final void initialize() {

	/* Init all robot-hardware corresponding objects. */
	imesLBR = (LBR) ServoMotionUtilities.locateLBR(getContext());
	getLogger().fine("robot object successfully created.");
	imesTool.addDefaultMotionFrame("TCP", toolTCP);
	imesTool.attachTo(imesLBR.getFlange());
	getLogger().fine("Tool attached to the robot object.");

	/* Reset Sunrise controller and ack possible errors. */
	ServoMotionUtilities.resetControllerAndKILLALLMOTIONS(imesLBR);
	ServoMotionUtilities.acknowledgeError(imesLBR);
	getLogger().fine("Resetted sunrise controller and acked all errors.");

	/*
	 * Check load data and then move to initial position. User interaction
	 * via the smartPad is needed therefore.
	 */
	getLogger().fine("Checking load data...");
	if (!SmartServo.validateForImpedanceMode(imesLBR)) {
	    getLogger().error("Validation of load data failed.");
	    throw new IllegalStateException("Load data is incorrect.");
	} else {
	    getLogger().info("Load data is validated succesfully.");
	}

	getLogger().fine("Show SmartPad dialog.");
	final int answerOnDialog = this.getApplicationUI().displayModalDialog(
		ApplicationDialogType.WARNING,
		"Robot will move to initial joint position ("
			+ INITIAL_ROBOT_POSE.toString() + ") if prompted!.",
		"OK");
	getLogger().fine("SmartPad dialog returned.");
	if (answerOnDialog == 0) {
	    getLogger().info("Dialog prompted by user.");
	    imesTool.move(ptp(INITIAL_ROBOT_POSE));
	    getLogger().info("Robot moved to initial position.");
	} else {
	    getLogger().error("SmartPad dialog cancelled.");
	    throw new IllegalStateException(
		    "Robot cannot move to intitial pose, because user dialog was cancelled.");
	}

	/*
	 * Define and parameterize the control mode for the state machine
	 * execution.
	 */
	getLogger().fine("Parameterizing the control mode...");
	controlMode = new CartesianImpedanceControlMode();
	paramCartesianImpedanceMode(controlMode);
	getLogger().info(
		controlMode.getClass().getTypeName()
			+ " set for state machine.");

	/*
	 * Set up all components for state machine execution.
	 */
	getLogger().fine("Initializing smart servo...");
	initSmartServo();
	getLogger().fine("Smart servo initialized.");

	getLogger().fine(
		"Initializing slicer control and slicer "
			+ "visualization threads...");
	initInterfaceThreads();
	getLogger().fine(
		"slicer control and slicer visualization threads initialized.");

	getLogger().fine("Initializing state machine...");
	initStateMachine();
	getLogger().fine("state machine initialized.");

    }

    /**
     * Parametrizes the cartesian control mode according to the defined
     * constants
     */
    private void paramCartesianImpedanceMode(IMotionControlMode mode) {
	((CartesianImpedanceControlMode) mode).parametrize(CartDOF.TRANSL)
		.setStiffness(TRANSL_STIFF);
	((CartesianImpedanceControlMode) controlMode).parametrize(CartDOF.ROT)
		.setStiffness(ROT_STIFF);
	((CartesianImpedanceControlMode) mode)
		.setNullSpaceStiffness(NULLSP_STIFF);
	((CartesianImpedanceControlMode) mode).setMaxPathDeviation(PATH_DEV_X,
		PATH_DEV_Y, PATH_DEV_Z, PATH_DEV_A, PATH_DEV_B, PATH_DEV_C);
    }

    /**
     * Initializes the slicer control interface thread and the slicer
     * visualization interface thread.
     */
    private void initInterfaceThreads() {
	slicerControlIf = new LWRStateMachineInterface();
	slicerControlIf.setPriority(6);
	slicerControlIf.debugInfos = true;
	// Setting the port for the Control Interface supported ports are 49001
	// to 49005. Default Value is 49001
	slicerControlIf.port = 49001;
	slicerControlIf.millisectoSleep = 20;

	slicerVisualIf = new LWRVisualizationInterface();
	slicerVisualIf.setPriority(5);
	slicerVisualIf.datatype = LWRVisualizationInterface.VisualIFDatatypes.JOINTSPACE;

	slicerVisualIf.DebugInfos = true;
	// Setting the port for the Visualization Interface supported ports are
	// 49001 to 49005. Default Value is 49002
	slicerVisualIf.port = 49002;
	slicerVisualIf.millisectoSleep = 25;

	slicerVisualIf.jntPose_StateM = smartServoRuntime
		.getAxisQMsrOnController();
	slicerVisualIf.cartPose_StateM = imesStatemachine.curPose;

    }

    /**
     * Initializes the state machine.
     */
    private void initStateMachine() {
	imesStatemachine = new LWRStatemachine();
	imesStatemachine.StartVisual = true;

	MatrixTransformation currentPose = MatrixTransformation
		.of((ITransformation) imesTool.getDefaultMotionFrame());
	imesStatemachine.curPose = currentPose;
	imesStatemachine.cmdPose = currentPose;
	imesStatemachine.controlMode = controlMode;
	imesStatemachine.currentVisualIFDatatype = VisualIFDatatypes.ROBOTBASE;

    }

    /**
     * Initializes the smart servo interface.
     */
    private void initSmartServo() {
	// Initializing the SmartServo
	SmartServo aRealtimeMotion = new SmartServo(
		imesLBR.getCurrentJointPosition());
	aRealtimeMotion.useTrace(true);

	// Set the motion properties of all robot motions during state machine
	// execution.
	aRealtimeMotion.setJointAccelerationRel(VEL);
	aRealtimeMotion.setJointVelocityRel(ACC);

	System.out.println("Starting SmartServo Realtime Motion in "
		+ controlMode.getClass().getName());

	// Set the control mode as member of the realtime motion
	imesTool.getDefaultMotionFrame().moveAsync(
		aRealtimeMotion.setMode(controlMode));

	// Fetch the Runtime of the Motion part
	// NOTE: the Runtime will exist AFTER motion command was issued
	smartServoRuntime = aRealtimeMotion.getRuntime();
	smartServoRuntime.setMinimumTrajectoryExecutionTime(TRAJ_EXC_TIME);

	// Reading the current a couple of times for safety reasons
	smartServoRuntime.updateWithRealtimeSystem();
	ThreadUtil.milliSleep(MS_TO_SLEEP);
	smartServoRuntime.updateWithRealtimeSystem();
	ThreadUtil.milliSleep(MS_TO_SLEEP);
    }

    /**
     * In this function the communication with the robot via RealTimePTP, the
     * communication with the Visualization and Control Software (e.g. 3D
     * Slicer, Matlab) and the State machine itself are operated.
     */
    @Override
    public final void run() {
	String LastPrintedError = "";
	String ErrorMessage = "";
	boolean VISUAL_ON = false;
	boolean StatemachineRun = true;
	int i = 0;
	long curTime = 0;
	int curTime_nanos = 0;
	long startTimeStamp = System.nanoTime();
	long curTime_millis = 0;
	JointPosition initialPosition = imesLBR.getCurrentJointPosition();

	getLogger().info("Starting Thread for state control communication ");
	slicerControlIf.start();

	StatisticTimer timing = new StatisticTimer();

	// Hauptschleife
	while (StatemachineRun && i < N_OF_RUNS) {

	    try {

		/*
		 * Start timer and statistic timer.
		 */
		startTimeStamp = (long) (System.nanoTime());
		OneTimeStep aStep = timing.newTimeStep();

		/* Update with controller of LWR. */
		{

		    try {
			smartServoRuntime.updateWithRealtimeSystem();
			// Get the measured position in cartesian pose
			imesStatemachine.curPose = MatrixTransformation
				.of((ITransformation) imesTool
					.getDefaultMotionFrame());

			// TODO auslagern in updatePose()
			imesStatemachine.curJntPose = smartServoRuntime
				.getAxisQMsrOnController();
			imesStatemachine.TCPForce = smartServoRuntime
				.getExtForceVector();
			imesStatemachine.PoseUID++;
		    } catch (Exception e) {
			ErrorMessage = "Error: Failed Update with RealtimeSystem!!";
			// TODO Exception handling später anpacken
		    }
		}

		if (slicerVisualIf.VisualRun) {

		    try {
			final boolean semaReqResult = slicerVisualIf.VisualSemaphore
				.tryAcquire(1, TimeUnit.MILLISECONDS);
			if (semaReqResult) {
			    slicerVisualIf.PoseUID = imesStatemachine.PoseUID;
			    slicerVisualIf.TCPForce = imesStatemachine.TCPForce;
			    slicerVisualIf.SendTCPForce = true;
			    updatePose();
			    slicerVisualIf.VisualSemaphore.release();
			}

			else {
			    ErrorMessage = "Error: Couldn't acquire VisualIF Semaphore!!";
			    // TODO exception handling einbauen.
			}
		    } catch (InterruptedException e) {
			e.printStackTrace();
			// TODO reaktion festlegen.
		    }
		}

		if (imesStatemachine.StartVisual && VISUAL_ON == false) {

		    updatePose();
		    slicerVisualIf.jntPose_StateM = initialPosition;
		    slicerVisualIf.VisualActive = true;
		    // Start the Visualization thread
		    slicerVisualIf.start();
		    VISUAL_ON = true;

		}

		else if (imesStatemachine.StartVisual && VISUAL_ON
			&& !slicerVisualIf.VisualActive) {
		    /*
		     * if Visualization interface is started, not active but is
		     * set active Change VisualActive to true. Thereby, the pose
		     * is send to the visualization.
		     */
		    slicerVisualIf.VisualActive = true;

		} else if (!imesStatemachine.StartVisual
			&& slicerVisualIf.VisualActive) {
		    /*
		     * if the visualization is running and the the Start Visual
		     * flag is false and the interface is still active Set the
		     * VisualACtive flag to false - thereby, no more data is
		     * send to the visualization.
		     */
		    slicerVisualIf.VisualActive = false;

		}

		// If SlicerControl Interface Thread is running...
		if (slicerControlIf.ControlRun) {

		    i = 0;
		    // Try to read new command String from SlicerControl (Alive)
		    // Thread
		    // TODO Austausch könnte über pipes geschehen.
		    try {
			boolean semaAcquired = slicerControlIf.controlSemaphore
				.tryAcquire(1, TimeUnit.MILLISECONDS);
			if (semaAcquired) {
			    imesStatemachine.CmdIGTmessage = slicerControlIf.CMD_StateM;
			    imesStatemachine.IGTLdatatype = slicerControlIf.IGTLdatatype;
			    imesStatemachine.UID = slicerControlIf.UID;
			    if (slicerControlIf.transformReceived
				    && !imesStatemachine.TransformRecieved) {
				imesStatemachine.TransformRobotImage = slicerControlIf.TransformImageRobot;
				imesStatemachine.TransformRecieved = true;
			    }
			    slicerControlIf.controlSemaphore.release();
			} else {
			    getLogger()
				    .fine("Acquiring of semaphore for setting state machine parameters failed!");
			}

		    } catch (InterruptedException e) {
			ErrorMessage = "Couldn't acquire Semaphore!!";
			getLogger().error(
				"Interrupted during waiting on semaphore", e);

		    }
		}

		else { // if it is not to Error handling
		    imesStatemachine.ErrorCode = 2;
		    ErrorMessage = "Slicer Control Interface not Alive...";
		    getLogger().error("Slicer control interface isn't running");
		    i++;
		}
		// Check if there is a Transition Request and in that case
		// Change the state and interpret the command parameters by
		// calling the function InterpretCommandString of the Current
		// State

		imesStatemachine.CheckTransitionRequest();// TODO ein check
							  // sollte immer! einen
							  // Rückgabewert
							  // liefern.

		// If the State has changed print the new State
		if (imesStatemachine.InitFlag) {
		    System.out.println("Robot State has Changed to:"
			    + imesStatemachine.RobotState.name());

		}

		// Check on Communication Quality
		if (slicerControlIf.ErrorCode == 18) { // Bad Communication
		    // Quality!!
		    // TODO was ist nun die Reaktion auf den Fehler? Wird
		    // Programm geschlossen?
		    imesStatemachine.ErrorCode = 18;
		    imesStatemachine.ErrorMessage = "ERROR: State Control Interface Bad Communication quality setting robot State to IDLE";
		    imesStatemachine.ErrorFlag = true;
		    slicerControlIf.ErrorCode = 0;

		}

		// Print Error messages if there where any Errors
		imesStatemachine.ErrorHandler(true);

		// Calculating the new control Param and Change the parameters
		imesStatemachine.CalcControlParam(); // TODO eine calc methode
		// sollte immer etwas
		// zurückgeben!

		// Change the control mode settings of the robot and send a new
		// Destination pose
		try {
		    smartServoRuntime
			    .changeControlModeSettings(imesStatemachine.controlMode);
		    smartServoRuntime.setDestination(imesStatemachine.cmdPose);
		} catch (Exception e) {
		    ErrorMessage = "Error: Failed to change Realtime Settings!!";
		    // TODO Reaktion festlegen
		}

		// Defining the Acknowledgement String for Control Interface
		imesStatemachine.SetACKPacket();// TODO Set im Methodenname
		// deutet auf Parameter hin!

		if (slicerControlIf.ControlRun) {
		    try {
			slicerControlIf.controlSemaphore.tryAcquire(1,
				TimeUnit.MILLISECONDS);
			// try to update the ACK String for the ControlIF Thread
			slicerControlIf.ACK_StateM = imesStatemachine.AckIGTmessage;
			slicerControlIf.controlSemaphore.release();
		    } catch (InterruptedException e) {
			ErrorMessage = "Error: Couldn't Acquire ControlIF Semaphore!!";
		    }
		}

		if (!ErrorMessage.equals(LastPrintedError)) {
		    getLogger().fine(ErrorMessage);
		    LastPrintedError = ErrorMessage;
		}

		cyclicSleep(startTimeStamp, curTime_millis, curTime,
			curTime_nanos);

		// Overall timing end
		aStep.end();
		if (imesStatemachine.End) {
		    StatemachineRun = false;

		}
	    } catch (InterruptedException e) {

		System.out.println(e);
		e.printStackTrace();
		slicerControlIf.ControlRun = false;
		slicerVisualIf.VisualRun = false;
	    }

	} // end while

	printFinalInfos(timing);

    }

    // Stopping the Control Interface thread and the visualization thread

    private void printFinalInfos(final StatisticTimer loopTimer) {
	// Print the timing statistics
	getLogger().info(
		"Statistic Timing of Statemachine interface thread "
			+ slicerControlIf.SMtiming);
	getLogger().info(
		"UID miss: " + slicerControlIf.UIDmiss + " UIDrepeats: "
			+ slicerControlIf.UIDrepeatNum + "(max: "
			+ slicerControlIf.UIDrepeat_max + ")");
	getLogger().info(
		"Statistic Timing of Visualisation interface thread "
			+ slicerVisualIf.Visualtiming);
	getLogger().info("PoseUID miss: " + slicerVisualIf.PoseUIDOldCount);
	getLogger().info("Statistic Timing of Statemachine Mean:" + loopTimer);

	getLogger().info(
		"Displaying final states after loop "
			+ controlMode.getClass().getName());
	smartServoRuntime.setDetailedOutput(1);

	if (loopTimer.getMeanTimeMillis() > MS_TO_SLEEP + 5) {
	    getLogger().info(
		    "Statistic Timing is unexpected slow, "
			    + "you should try to optimize TCP/IP Transfer");
	    getLogger()
		    .info("Under Windows, you should play with the registry, "
			    + "see the e.g. the RealtimePTP Class javaDoc for details");
	}
    }

    private void cyclicSleep(final long startTime, long currentTime_millis,
	    long currentTime, int currentTimeNanos) throws InterruptedException {
	// Set the Module in Sleep mode for stability enhancement
	// TODO Code zur Zeitumrechnung wurde mehrfach implementiert!
	// TODO vereinfachen mit TimeUnit
	currentTime = (long) ((System.nanoTime() - startTime));
	currentTime_millis = TimeUnit.NANOSECONDS.toMillis(currentTime);
	currentTimeNanos = (int) (currentTime % 1000000);
	if (currentTime_millis < MS_TO_SLEEP - 2) {
	    Thread.sleep(MS_TO_SLEEP - 2 - currentTime_millis,
		    999999 - currentTimeNanos);

	}
    }

    private void updatePose() {

	slicerVisualIf.cartPose_StateM = imesStatemachine.curPose;
	slicerVisualIf.datatype = imesStatemachine.currentVisualIFDatatype;
	switch (imesStatemachine.currentVisualIFDatatype) {
	case IMAGESPACE: {
	    if (imesStatemachine.TransformRecieved) {
		slicerVisualIf.T_IMGBASE_StateM = imesStatemachine.TransformRobotImage;
	    }
	}

	case JOINTSPACE: {
	    slicerVisualIf.jntPose_StateM = imesStatemachine.curJntPose;
	}
	default:
	    break;

	}

    }

    /**
     * In this function the communication with the robot via RealTimePTP, the
     * communication with the Visualization and Control Software (e.g. 3D
     * Slicer, Matlab) and the State machine it self are operated. TODO
     * Parameterbeschreibung.
     */
    // TODO Methode ist zu lang und schwer zu verstehen.
    private void runRealtimeMotion(IMotionControlMode controlMode) {

    }

    /**
     * Auto-generated method stub. Do not modify the contents of this method.
     */
    public static void main(final String[] args) {
	StateMachineApplication app = new StateMachineApplication();
	app.runApplication();
    }

    /**
     * Stops all running communication threads.
     */
    public final void dispose() {

	slicerControlIf.ControlRun = false;
	slicerVisualIf.VisualRun = false;

	// Stop the motion
	final boolean motionStopped = smartServoRuntime.stopMotion();
	if (!motionStopped) {
	    getLogger().error("Cannot stop motion of smartServoRuntime.");
	}

	if (slicerControlIf != null) {
	    slicerControlIf.finalize();
	}
	if (slicerVisualIf != null) {
	    slicerVisualIf.finalize();

	}

    }
}
