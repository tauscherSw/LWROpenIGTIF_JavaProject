/*=========================================================================

  Program:   StateMachineApplication
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

package de.uniHannover.imes.igtIf.application;

import java.io.File;
import java.io.IOException;


import de.uniHannover.imes.igtIf.logging.DebugLogger;
/*AUFGABEN:
 -Globaler DebugLoggingMechanismus zu unübersichtlich? Mglkt. unterschiedliche Klassen/Logfiles bilden.
 -Übergabe des Loggers im kompletten igtif Packet unnötig weil über java.util global erreichbar. Alternative überlegen. z.b. Erlaubnis zum loggen über flag 
 -Globale Erlaubnis debug zu loggen über ein Flag schlecht. besser gezielt einstellen, oder guten log viewer suchen.
 -log output anschließend überarbeiten.
 -GitHub Dokumentation -> Projektpage
 -Unit Test für Schnittstellen IGTL
 */
import de.uniHannover.imes.igtIf.logging.FileLogger;
import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine;
import de.uniHannover.imes.igtIf.stateMachine.LwrStatemachine.OpenIGTLinkErrorCode;
import de.uniHannover.imes.igtIf.util.FileSystemUtil;
import de.uniHannover.imes.igtIf.util.SleepUtil;
import de.uniHannover.imes.igtIf.util.StatisticalTimer;
import de.uniHannover.imes.igtIf.communication.ExceptionHandlerComThreads;
import de.uniHannover.imes.igtIf.communication.control.CommunicationDataProvider;
import de.uniHannover.imes.igtIf.communication.control.ControlThread;
import de.uniHannover.imes.igtIf.communication.visualization.VisualizationThread;
import de.uniHannover.imes.igtIf.communication.visualization.VisualizationThread.VisualIFDatatypes;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.physics.Inertia;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

/**
 * This is an example robot application for an LBR 4 and a Sunrise controller
 * using the LWRVisualization and LWRStatemachineInterface. For the
 * Communication with the robot the SmartServo interface is used. For further
 * information on this topic see the SmartServo documentation. To use the
 * Visualization- and StateControlInterface class you need to declare the object
 * in the main program of your robotAPI e.g the
 * runRealtimeMotion(IMotionControlMode controlMode) function.
 * 
 * <pre>
 * <code>
 * {@code
 * //Flag to indicate if the Visualization is active or not
 * boolean VisualON=false;
 * 
 * // To set the Visualization active at the start of the program just set the StartVisual flag of the lwrStatemachine Object imesStatemachine to true,
 * // e.g. imesStatemachine.StartVisual=true. Else this flag is set if the StateControl sends the Command "Visual;true;img/rob/jnt"
 * //imesStatemachine.StartVisual=true;
 * 
 * // Declaration of a LWRStateMachineInterface Object for the communication with a State Control using OpenIGTLink
 * LWRStateMachineInterface SlicerControlIf = new LWRStateMachineInterface();
 * 
 * //Declaration of a LWRVisualizationInterface Object for the communication with Visualization using OpenIGTLink e.g. 3D Slicer OpenIGTIF
 * LWRVisualizationInterface SlicerVisualIF = new LWRVisualizationInterface();
 * 
 * //Setting the port for the Control Interface supported ports are 49001 to 49005. Default Value is 49001
 * SlicerControlIf.port =49001;
 * 
 * //Setting the port for the Visualization Interface supported ports are 49001 to 49005. Default Value is 49002
 * SlicerVisualIF.port = 49002;
 * }
 * </code>
 * </pre>
 * 
 * After this the SmartServo Motion needs to be initialized (see SmartServo
 * Documentation) and the SlicerControl thread is started.
 * 
 * <pre>
 * {@code
 * //initializing and starting of the AliveThread for the Communication with the State controller
 * SlicerControlIf.start();
 * }
 * </pre>
 * 
 * After the current position of the robot was read from the SmartServoRuntime
 * the current Position of the imesStatemachine is initialized with these
 * values. *
 * 
 * <pre>
 * {@code
 * //Setting the imes State machine member variables such as the control Mode
 * imesStatemachine.curPose= MatrixTransformation.of(SmartServoRuntime.getCartFrmMsrOnController());
 * imesStatemachine.controlMode = controlMode;
 * imesStatemachine.cmdPose = MatrixTransformation.of(SmartServoRuntime.getCartFrmMsrOnController());
 * 
 * //Initialize some of the Visualization Interface member variables
 * SlicerVisualIF.jntPose_StateM = SmartServoRuntime.getAxisQMsrOnController();
 * SlicerVisualIF.cartPose_StateM = imesStatemachine.curPose;
 * }
 * </pre>
 * 
 * When these initializing routine is done the main loop is entered. The loop
 * stopps when: - the command Shutdown/End/Quit were received from the state
 * control - when there was no packet received from the state control for
 * _numRuns loops
 * 
 * 
 * @author Sebastian Tauscher
 * @version 0.2
 * @see
 */
public class StateMachineApplication extends RoboticsAPIApplication {

    // **************************Constants**********************/

    /** Time in milliseconds for waiting for communication threads to end. */
    private static final int JOIN_TIME_THREADS = 2500;

    /**
     * Acceleration of robot movements during state-machine execution. Value in
     * %.
     */
    private static final double ACC = 1;

    /**
     * Definition of the initial robot position before the state-machine starts
     * working.
     */
    private static final JointPosition INITIAL_ROBOT_POSE = new JointPosition(
	    0.0, Math.toRadians(30), 0., -Math.toRadians(60), 0.,
	    Math.toRadians(90), 0.);
    /**
     * Maximum allowed deviation of the statistic timer of the main loop.
     */
    private static final int MAXIMUM_TIMING_DEVIATION_MS = 5;

    /**
     * Cyclic time of each loop of the main (state machine) thread.
     */
    private static final int MS_TO_SLEEP = 10;
    /**
     * number of loops to run with out any communication with the state control.
     */
    private static final int N_OF_RUNS = 500;

    /** Damping of the cartesian impedance control mode. */
    private static final double DAMPING = 0.7;

    /**
     * The nullspace stiffness (rotational), when the robot is moving in
     * cartesian impedance mode. Value in Nm/rad.
     */
    private static final double NULLSP_STIFF = 200.;
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
     * The rotational stiffness, when the robot is moving in cartesian impedance
     * mode. Value in Nm/rad.
     */
    private static final double ROT_STIFF = 300;

    /**
     * Minimum trajectory execution time during state-machine execution. Value
     * in seconds.
     */
    private static final double TRAJ_EXC_TIME = 5e-03;
    /**
     * The translational stiffness, when the robot is moving in cartesian
     * impedance mode. Value in N/m.
     */
    private static final double TRANSL_STIFF = 2000;
    /**
     * Velocity of robot movements during state-machine execution. Value in %.
     */
    private static final double VEL = 1;

    // **************************Flags**************************/
    /** Flag indicating if stateMachine is runnable. */
    private boolean stateMachineRun = true;
    
    /**
     * Turn this flag to true if debugging should be enabled. A debug-Logging
     * output will be saved in a File-Logger in user.home.
     */
    public static final boolean DEBUG_MODE = true;

    // **************************Components*********************/
    /** Debug File logger. */
    private ITaskLogger log;

    /** Statistical timer for main loop. */
    private StatisticalTimer timer;
    /**
     * Control mode for movements during state control.
     */
    private IMotionControlMode controlMode;

    /** The robot object for controlling robot movements. */
    private LBR imesLBR;

    /**
     * Object of the State machine class.
     * 
     * @see LwrStatemachine
     */
    private LwrStatemachine imesStatemachine;

    /**
     * The tool object describing the physical properties of the tool attached
     * to the robot's flange.
     */
    // private final Tool imesTool = new Tool("Imes Tool", new LoadData(0.4,
    // MatrixTransformation.ofTranslation(-5,0,50), Inertia.ZERO));
    // TODO dummy tool used for debugging
    private final Tool imesTool = new Tool("Imes Tool", new LoadData(0.0,
	    MatrixTransformation.ofTranslation(0, 0, 0), Inertia.ZERO));

    /**
     * Object of the state machine interface class for the communication with a
     * state control software using the OpenIGTLink protocol.
     * 
     * @see LWRStateMachineInterface
     */
    private ControlThread controlThread;

    /**
     * Logs the uncaught exceptions in the underlying threads (for
     * communication).
     */
    private ExceptionHandlerComThreads threadExcHdl;

    /**
     * The provider for the communication data, exchanged with OpenIGTLink. This
     * class provides access to all the commands, uids and so on received from
     * an external OpenIGTLink client.
     */
    private CommunicationDataProvider comDataProvider;

    /**
     * Object of the visualization interface class for the communication with a
     * visualization software using the OpenIGTLink protocol.
     * 
     * @see LWRVisualizationInterface
     */
    private VisualizationThread visualizationThread;

    /** Sunrise specific interface to control the robot's movements. */
    private ISmartServoRuntime smartServoRuntime;

    /**
     * Definition of the tool-center-point.
     */
    private final MatrixTransformation toolTCP = MatrixTransformation
	    .ofTranslation(-40, 10, 207);

    // ***************************Methods***********************/

    /**
     * Initializes the slicer control interface thread and the slicer
     * visualization interface thread.
     * 
     * @throws IOException
     *             when setup of network-communication fails.
     */
    private void initInterfaceThreads() throws IOException {

	controlThread = new ControlThread(imesStatemachine, comDataProvider,
		log);
	controlThread.setUncaughtExceptionHandler(threadExcHdl);

	visualizationThread = new VisualizationThread(comDataProvider, log);
	visualizationThread.setUncaughtExceptionHandler(threadExcHdl);
	visualizationThread.updateData();

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

	log.fine("Starting SmartServo Realtime Motion in "
		+ controlMode.getClass().getSimpleName());

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
     * Initializes the state machine.
     */
    private void initStateMachine() {
	imesStatemachine = new LwrStatemachine(comDataProvider,log);
	imesStatemachine.startVisual = true;

	comDataProvider.readNewRobotData();
	imesStatemachine.cmdPose = comDataProvider.getCurRobotDataSet()
		.getCurPose();
	imesStatemachine.controlMode = controlMode;
	imesStatemachine.setVisualIfDatatype(VisualIFDatatypes.ROBOTBASE);

    }

    /**
     * Parametrizes the cartesian control mode according to the defined
     * constants.
     * 
     * @param mode
     *            The control-mode-object, which has to be parameterized.
     */
    private void paramCartesianImpedanceMode(final IMotionControlMode mode) {
	((CartesianImpedanceControlMode) mode).parametrize(CartDOF.TRANSL)
		.setStiffness(TRANSL_STIFF);
	((CartesianImpedanceControlMode) controlMode).parametrize(CartDOF.ROT)
		.setStiffness(ROT_STIFF);
	((CartesianImpedanceControlMode) mode)
		.setNullSpaceStiffness(NULLSP_STIFF);
	((CartesianImpedanceControlMode) mode).parametrize(CartDOF.ALL)
		.setDamping(DAMPING);
	((CartesianImpedanceControlMode) mode).setMaxPathDeviation(PATH_DEV_X,
		PATH_DEV_Y, PATH_DEV_Z, PATH_DEV_A, PATH_DEV_B, PATH_DEV_C);
    }

    /**
     * Prints timing statistics and communication parameters.
     * 
     */
    private void printFinalInfos() {
	// Print the timing statistics
	log.info("Displaying final states after loop "
		+ controlMode.getClass().getName());
	smartServoRuntime.setDetailedOutput(1);
	log.info(timer.getOverallStatistics());

	if (timer.getMeanTimeMillis() > MS_TO_SLEEP
		+ MAXIMUM_TIMING_DEVIATION_MS) {
	    log.info("Statistic Timing is unexpected slow, "
		    + "you should try to optimize TCP/IP Transfer");
	    log.info("Under Windows, you should play with the registry, "
		    + "see the e.g. the RealtimePTP Class javaDoc for details");
	}
    }

    /**
     * In this function the robot, tool etc are initialized.
     **/
    @Override
    public final void initialize() {

	/* Initialize debug logger if needed. */
	if (DEBUG_MODE) {
	    log = DebugLogger.getInstance();
	    getLogger().warn(
		    "DEBUG LOGGER is enabled. All logging-output will be "
			    + "directed to the file: "
			    + DebugLogger.DEBUG_LOGFILE.getAbsolutePath());
	} else {
	    log = getLogger();
	}

	/* Load swig library. */
	FileSystemUtil.loadSwigDll();

	/* Init all robot-hardware corresponding objects. */
	imesLBR = (LBR) ServoMotionUtilities.locateLBR(getContext());
	log.fine("robot object successfully created.");
	imesTool.addDefaultMotionFrame("TCP", toolTCP);
	log.warn("No tool configured in this application");
	imesTool.attachTo(imesLBR.getFlange());
	log.fine("Tool attached to the robot object.");

	/* Reset Sunrise controller and ack possible errors. */
	ServoMotionUtilities.resetControllerAndKILLALLMOTIONS(imesLBR);
	ServoMotionUtilities.acknowledgeError(imesLBR);
	log.fine("Resetted sunrise controller and acked all errors.");

	/*
	 * Check load data and then move to initial position. User interaction
	 * via the smartPad is needed therefore.
	 */
	log.fine("Checking load data...");
	if (!SmartServo.validateForImpedanceMode(imesLBR)) {
	    log.error("Validation of load data failed.");
	    throw new IllegalStateException("Load data is incorrect.");
	} else {
	    log.info("Load data is validated succesfully.");
	}

	log.fine("Show SmartPad dialog.");
	final int answerOnDialog = this.getApplicationUI().displayModalDialog(
		ApplicationDialogType.WARNING,
		"Robot will move to initial joint position ("
			+ INITIAL_ROBOT_POSE.toString() + ") if prompted!.",
		"OK");
	log.fine("SmartPad dialog returned.");
	if (answerOnDialog == 0) {
	    log.info("Dialog prompted by user.");
	    imesTool.move(ptp(INITIAL_ROBOT_POSE));
	    log.info("Robot moved to initial position.");
	} else {
	    log.error("SmartPad dialog cancelled.");
	    throw new IllegalStateException(
		    "Robot cannot move to intitial pose, "
			    + "because user dialog was cancelled.");
	}

	/*
	 * Define and parameterize the control mode for the state machine
	 * execution.
	 */
	log.info("Parameterizing the control mode...");
	controlMode = new CartesianImpedanceControlMode();
	paramCartesianImpedanceMode(controlMode);
	log.info(controlMode.getClass().getSimpleName()
		+ " set for state machine.");

	/*
	 * Set up all components for state machine execution.
	 */
	log.info("Initializing communication data provider...");
	comDataProvider = new CommunicationDataProvider(imesLBR, log);
	log.info("Communication data provider initialized.");

	log.info("Initializing state machine...");
	initStateMachine();
	log.info("state machine initialized.");

	threadExcHdl = new ExceptionHandlerComThreads(log);
	log.info("Initializing smart servo...");
	initSmartServo();
	log.info("Smart servo initialized.");

	log.info("Initializing slicer control and slicer "
		+ "visualization threads...");
	try {
	    initInterfaceThreads();
	} catch (IOException e) {
	    throw new IllegalStateException(
		    "Cannot initialize interfacing threads. ", e);
	}
	log.info("slicer control and slicer visualization threads initialized.");

    }

    /**
     * In this function the communication with the robot via RealTimePTP, the
     * communication with the Visualization and Control Software (e.g. 3D
     * Slicer, Matlab) and the State machine itself are operated.
     */
    @Override
    public final void run() {
	try {
	    int i = 0;

	    timer = new StatisticalTimer(MS_TO_SLEEP); // timing statistics for
	    // following loop.
	    long startTimeStamp;

	    log.info("Starting Thread for state control communication.");
	    controlThread.start();
	    log.info("Starting Thread for visualization communication, "
		    + "but yet not enabled to send data.");
	    visualizationThread.start();

	    // Main loop
	    log.info(this.getClass().getName() + " is entering the main loop");

	    while (stateMachineRun && i < N_OF_RUNS) {

		log.fine(this.getClass().getName() + " begins the main loop");
		try {

		    /*
		     * Start timer and statistic timer.
		     */
		    startTimeStamp = System.nanoTime();
		    timer.loopBegin();

		    /* Update with controller of LWR. */
		    try {
			log.fine(this.getClass().getName()
				+ " updates the smart servo runtime.");
			ThreadUtil.milliSleep(MS_TO_SLEEP);
			smartServoRuntime.updateWithRealtimeSystem();

		    } catch (Exception e) {

			log.warn(this.getClass().getName()
				+ " failed to update the smart servo runtime. "
				+ "Reinitializing smartServo...", e);
			initSmartServo();
		    }
		    log.fine(this.getClass().getName()
			    + " induces an update of the current robot data.");
		    comDataProvider.readNewRobotData();

		    if (visualizationThread.getCondWork()) {

			log.fine(this.getClass().getName()
				+ " induces an update of the data of "
				+ visualizationThread.getClass().getName()
				+ " thread.");
			visualizationThread.setSenderConfiguration(null, true);
			visualizationThread.updateData();
		    }

		    /*
		     * Control the sending of the visualization data by the
		     * visualization thread.
		     */
		    if (imesStatemachine.startVisual
			    && !visualizationThread.isAlive()) {

			log.fine(this.getClass().getName() + " enables the "
				+ visualizationThread.getClass().getName()
				+ " thread.");

			visualizationThread.setCondWork(true);

		    } else if (!imesStatemachine.startVisual
			    && visualizationThread.isAlive()) {
			log.fine(this.getClass().getName() + " disables the "
				+ visualizationThread.getClass().getName()
				+ " thread.");
			visualizationThread.setCondWork(false);
		    }

		    // If SlicerControl Interface Thread is running...
		    if (controlThread.isAlive()) {

			// update the data in the state machine and reset error
			// counter.
			log.fine(this.getClass().getName()
				+ " induces a update of the statemachine data.");
			i = 0;
			imesStatemachine.updateStateControlData();

		    } else {
			// control thread wasn't alive -> restarting
			imesStatemachine.ErrorCode = OpenIGTLinkErrorCode.UnknownError;
			log.error(this.getClass().getName()
				+ " has detected, that the communication in "
				+ controlThread.getClass().getName()
				+ " isnt running."
				+ "Slicer Control Interface not Alive...");
			i++;
			if (stateMachineRun) {
			    log.warn(this.getClass().getName()
				    + " has to be restarted.");
			    try {
				controlThread = new ControlThread(
					imesStatemachine, comDataProvider, log);
			    } catch (IOException e) {
				log.error(this.getClass().getName()
					+ " cannot restart "
					+ controlThread.getClass()
						.getSimpleName()
					+ " and thus terminates the statemachine.");
				stateMachineRun = false;
			    }
			    controlThread.start();
			}

		    }

		    /*
		     * Check if there is a Transition Request and in that case
		     * Change the state and interpret the command parameters by
		     * calling the function InterpretCommandString of the
		     * current state.
		     */
		    log.fine(this.getClass().getName()
			    + " induces a check of a transition request"
			    + " in the state machine.");
		    String oldState = imesStatemachine.getCurrentState()
			    .getClass().getSimpleName();
		    imesStatemachine.checkTransitionRequest();
		    // If the State has changed print the new State
		    if (imesStatemachine.stateChanged) {
			log.info("Robot State has changed from "
				+ oldState
				+ " to "
				+ imesStatemachine.getCurrentState().getClass()
					.getSimpleName());

		    }

		    // Print Error messages if there where any Errors
		    imesStatemachine.errHandler(true);

		    /*
		     * Calculating the new control Param and Change the
		     * parameters.
		     */
		    log.fine("Calculating new control parameters");
		    imesStatemachine.updateCtrlParam();

		    /*
		     * Change the control mode settings of the robot and send a
		     * new Destination pose.
		     */
		    try {
			log.fine("Setting control mode settings to "
				+ imesStatemachine.controlMode.toString());
			smartServoRuntime
				.changeControlModeSettings(imesStatemachine.controlMode);
			ThreadUtil.milliSleep(3);
			log.fine("Commanding new robot-pose "
				+ imesStatemachine.cmdPose.toString());

			smartServoRuntime
				.setDestination(imesStatemachine.cmdPose);

		    } catch (Exception e) {
			// log
			// .error("Cannot change control mode settings or command a new pose. Resetting smart servo",
			// e);
			// ServoMotionUtilities.resetControllerAndKILLALLMOTIONS(imesLBR);
			// ServoMotionUtilities.acknowledgeError(imesLBR);
			// initSmartServo();
		    }

		    // Defining the acknowledgment String for Control Interface
		    log.fine(this.getClass().getName()
			    + "aquires the acknowledgement packet from the state machine.");
		    imesStatemachine.setAckPacket();

		    if (controlThread.isAlive()) {
			// try to update the ACK String for the ControlIF Thread
			log.fine("Sending acknowledgement message");
		    }

		    /*
		     * sleep for a specified time (according to the loops
		     * iteration time).
		     */
		    SleepUtil.cyclicSleep(startTimeStamp, 2, MS_TO_SLEEP);

		    // Overall timing end
		    timer.loopEnd();
		    if (imesStatemachine.End) {
			log.info("State machine was stopped, ending main loop.");
			stateMachineRun = false;

		    }
		} catch (InterruptedException e) {

		    log.error(this.getClass().getName()
			    + " was interrupted in its main loop. "
			    + "All connections will be closed.", e);

		    controlThread.setCondWork(false);
		}

	    } // end while

	} finally {
	    // Print final infos.
	    log.info("Statistics for loop of main thread: ");
	    printFinalInfos();

	}

    }

    /**
     * Stops all running communication threads And stops the motion.
     */
    public final void dispose() {

	// Stop the motion
	final boolean motionStopped = smartServoRuntime.stopMotion();
	if (!motionStopped) {
	    log.error("Cannot stop motion of smartServoRuntime.");
	}

	// Stop all threads.
	controlThread.interrupt();
	visualizationThread.interrupt();
	try {
	    controlThread.join(JOIN_TIME_THREADS);
	    visualizationThread.join(JOIN_TIME_THREADS);
	} catch (InterruptedException e) {
	    log.error(
		    "Waiting for the ending of the communication threads was interrupted.",
		    e);
	}

	log.info(this.getClass().getSimpleName() + " was disposed properly.");
	super.dispose();

    }

    /**
     * Method is invoked by application server, when the state of this robotic
     * application changed. When state changed to stopping, the execution of the
     * main loop is interrupted.
     * 
     * @param state
     *            the new state of the robotics-application.
     */
    public final void onApplicationStateChanged(
	    final RoboticsAPIApplicationState state) {

	switch (state) {
	case STOPPING:
	    stateMachineRun = false;
	case MOTIONPAUSING:

	    /*
	     * Check if user wants to abort the statemachine via the smartpad by
	     * an ui-dialog.
	     */
	    log.fine("Show SmartPad dialog if abortion is desired.");
	    final int answerOnDialog = this.getApplicationUI()
		    .displayModalDialog(ApplicationDialogType.QUESTION,
			    "Abort the state machine?", "Yes", "No");
	    log.fine("SmartPad dialog returned.");
	    if (answerOnDialog == 0) {
		log.info("Aborting statemachine.");
		stateMachineRun = false;
	    } else {
		log.info("Continuing statemachine.");
	    }
	default:
	    break;
	}

    }

    /**
     * Auto-generated method stub. Do not modify the contents of this method.
     * 
     * @param args
     *            unused arguments.
     */
    public static void main(final String[] args) {
	StateMachineApplication app = new StateMachineApplication();
	app.runApplication();
    }

}
