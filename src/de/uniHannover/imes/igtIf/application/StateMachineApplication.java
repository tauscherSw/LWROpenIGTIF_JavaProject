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

import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

import de.uniHannover.imes.igtIf.logging.LwrIgtlLogConfigurator;
import de.uniHannover.imes.igtIf.logging.LwrIgtlLogConfigurator.LogForwardType;
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
 * This is an example robot application for an LWR 5-14kg and the Sunrise
 * controller, which installs a statemachine on the controller and communicates
 * via openIGTLink with external clients for statemachine control. The states
 * represent different robot behavior (commanded via SmartServo -> see KUKA
 * documentation for more detail) like gravity compensation and so on. For
 * further information and setup see {@linkplain
 * https://github.com/tauscherSw/LWROpenIGTIF_JavaProject/tree/master}
 * 
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

    /**
     * Defines how log messages are processed.
     */
    public static final LogForwardType DEBUG_LOG_FORWARD_TYPE = LogForwardType.Network;

    /** Flag to indicate if main loop is active. */
    private boolean mainLoopActive = false;

    // **************************Components*********************/

    /**
     * Logging mechanism provided by jdk. In case if debug flag is active, all
     * logging output will be directed to a logfile. Otherwise logging output
     * will be displayed on the smartpad.
     */
    private Logger logger;

    /** Configures a global accessible logger. */
    private LwrIgtlLogConfigurator logConfig;

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
    private final Tool imesTool = new Tool("Imes Tool", new LoadData(0.44,
	    MatrixTransformation.ofTranslation(-5, 0, 50), Inertia.ZERO));
    // TODO dummy tool used for debugging
    // private final Tool imesTool = new Tool("Imes Tool", new LoadData(0.0,
    // MatrixTransformation.ofTranslation(0, 0, 0), Inertia.ZERO));

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
     * In this function the robot, tool etc are initialized.
     **/
    @Override
    public final void initialize() {

	// Parameterize project logging mechanism
	logConfig = LwrIgtlLogConfigurator.getInstance();
	try {
	    logConfig.setup(getLogger(), DEBUG_MODE, DEBUG_LOG_FORWARD_TYPE);
	} catch (IOException e) {
	    getLogger().error("Setup of global logger failed.", e);
	}

	// get logger for this class
	logger = Logger.getLogger(LwrIgtlLogConfigurator.LOGGERS_NAME);

	// Beginning intialization process
	logger.info("Beginning initialization...");
	logger.entering(this.getClass().getName(), "initialize()");

	/* Load swig library. */
	FileSystemUtil.loadSwigDll();
	logger.finest("SWIG library loaded.");

	/* Reset Sunrise controller and ack possible errors. */
	ServoMotionUtilities.resetControllerAndKILLALLMOTIONS(imesLBR);
	ServoMotionUtilities.acknowledgeError(imesLBR);
	logger.finest("Resetted sunrise controller and acked all errors.");

	/* Initialize all robot-hardware corresponding objects. */
	imesLBR = (LBR) ServoMotionUtilities.locateLBR(getContext());
	logger.finest("Robot object successfully created.");
	imesTool.addDefaultMotionFrame("TCP", toolTCP);
	logger.warning("No tool configured in this application");
	imesTool.attachTo(imesLBR.getFlange());
	logger.finest("Tool attached to the robot object.");

	/*
	 * Check load data and then move to initial position. User interaction
	 * via the smartPad is needed therefore.
	 */
	logger.finest("Show SmartPad dialog about going to intial pose.");
	final int answerOnDialog = this.getApplicationUI().displayModalDialog(
		ApplicationDialogType.WARNING,
		"Robot will move to initial joint position ("
			+ INITIAL_ROBOT_POSE.toString() + ") if prompted!.",
		"OK");
	logger.finest("SmartPad dialog returned.");
	if (answerOnDialog == 0) {
	    logger.info("Dialog prompted by user.");
	    imesTool.move(ptp(INITIAL_ROBOT_POSE));
	    logger.info("Robot moved to initial position.");
	} else {
	    logger.severe("SmartPad dialog cancelled.");
	    throw new IllegalStateException(
		    "Robot cannot move to intitial pose, "
			    + "because user dialog was cancelled.");
	}

	logger.info("Checking load data...");
	if (!SmartServo.validateForImpedanceMode(imesLBR)) {
	    logger.severe("Validation of load data failed.");
	    throw new IllegalStateException("Load data is incorrect.");
	} else {
	    logger.info("Load data is validated succesfully.");
	}

	/*
	 * Define and parameterize the control mode for the state machine
	 * execution.
	 */
	controlMode = new CartesianImpedanceControlMode();
	paramCartesianImpedanceMode(controlMode);
	logger.finest("Control mode parametrized as "
		+ controlMode.getClass().getSimpleName());

	/*
	 * Set up all components for state machine execution.
	 */
	comDataProvider = new CommunicationDataProvider(imesLBR);
	logger.finest("Communication data provider initialized.");

	initStateMachine();
	logger.finest("state machine initialized.");

	threadExcHdl = new ExceptionHandlerComThreads();
	initSmartServo();
	logger.finest("Smart servo and thread-exception-handler initialized.");

	try {
	    initInterfaceThreads();
	} catch (IOException e) {
	    throw new IllegalStateException(
		    "Cannot initialize interfacing threads. ", e);
	}
	logger.finest("IGTL-interface threads initialized.");

	logger.info("Initialization finished.");
	logger.exiting(this.getClass().getName(), "initialize()");

    }

    /**
     * Initializes the slicer control interface thread and the slicer
     * visualization interface thread.
     * 
     * @throws IOException
     *             when setup of network-communication fails.
     */
    private void initInterfaceThreads() throws IOException {

	controlThread = new ControlThread(imesStatemachine, comDataProvider);
	controlThread.setUncaughtExceptionHandler(threadExcHdl);

	visualizationThread = new VisualizationThread(comDataProvider);
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

	logger.fine("Starting SmartServo Realtime Motion in "
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
	// smartServoRuntime.updateWithRealtimeSystem(); //TODO @TOBI
	// ThreadUtil.milliSleep(MS_TO_SLEEP);
    }

    /**
     * Initializes the state machine.
     */
    private void initStateMachine() {
	imesStatemachine = new LwrStatemachine(comDataProvider);
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
     * In this function the communication with the robot via RealTimePTP, the
     * communication with the Visualization and Control Software (e.g. 3D
     * Slicer, Matlab) and the State machine itself are operated.
     */
    @Override
    public final void run() {
	logger.entering(this.getClass().getName(), "run()");
	try {

	    // Initialize loop specific variables.
	    int controlThreadNotAlive = 0;
	    timer = new StatisticalTimer(MS_TO_SLEEP); // timing statistics for
	    // following loop.
	    long startTimeStamp;

	    controlThread.start();
	    logger.info("Thread for state control communication started.");
	    visualizationThread.start();
	    logger.info("Starting Thread for visualization communication, "
		    + "but yet not enabled to send data.");
	    logger.finest("Pre-main-loop-initialization finished.");

	    // Main loop
	    logger.info("State machine running...");
	    mainLoopActive = true;

	    while (stateMachineRun && controlThreadNotAlive < N_OF_RUNS) {

		logger.fine("Begin main loop");
		try {

		    /*
		     * Start timer and statistic timer.
		     */
		    startTimeStamp = System.nanoTime();
		    timer.loopBegin();

		    /* Update with controller of LWR. */
		    // TODO @TOBI weglassen
		    try {
			// ThreadUtil.milliSleep(MS_TO_SLEEP);
			smartServoRuntime.updateWithRealtimeSystem();

			logger.fine("Smart-servo-runtime updated.");

		    } catch (Exception e) {

			logger.log(Level.WARNING,
				"Failed to update the smart servo runtime.");
			// initSmartServo();
		    }

		    /* Collect new data from the robot. */
		    logger.fine("Reading new robot-data...");
		    comDataProvider.readNewRobotData();
		    logger.fine("New robot-data read.");

		    /* Send robot visualization data via openIGTL. */
		    if (visualizationThread.getCondWork()) {

			visualizationThread.setSenderConfiguration(null, true);
			visualizationThread.updateData();
			logger.fine("Data in visualization thread updated.");
		    }

		    /*
		     * Check if visualization thread should be enabled/disabled.
		     */
		    if (imesStatemachine.startVisual
			    && !visualizationThread.isAlive()) {

			visualizationThread.setCondWork(true);
			logger.info("Robot-visualization enabled.");

		    } else if (!imesStatemachine.startVisual
			    && visualizationThread.isAlive()) {
			visualizationThread.setCondWork(false);
			logger.info("Robot-visualization disabled.");
		    }

		    // If state-machine control thread is running...
		    if (controlThread.isAlive()) {
			logger.fine("Control thread is alive.");
			// update the data in the state machine and reset error
			// counter.
			logger.fine("Updating state control data...");
			controlThreadNotAlive = 0;
			imesStatemachine.updateStateControlData();
			logger.fine("Updated state control data.");

		    } else {
			// control thread wasn't alive -> restarting
			imesStatemachine.ErrorCode = OpenIGTLinkErrorCode.UnknownError;
			logger.severe("Control thread isn't alive...");
			controlThreadNotAlive++;
			if (stateMachineRun) {
			    logger.fine("Statemachine is still alive...");
			    logger.warning("Restarting control thread...");
			    try {
				controlThread = new ControlThread(
					imesStatemachine, comDataProvider);
			    } catch (IOException e) {
				logger.severe("Cannot restart "
					+ controlThread.getClass()
						.getSimpleName()
					+ " and thus terminates the statemachine.");
				stateMachineRun = false;
			    }
			    controlThread.start();
			    logger.info("Control thread restarted...");
			}

		    }

		    /*
		     * Check if there is a Transition Request and in that case
		     * Change the state and interpret the command parameters by
		     * calling the function InterpretCommandString of the
		     * current state.
		     */
		    logger.fine("Check for a transition requests in the state "
			    + "machine...");
		    String oldState = imesStatemachine.getCurrentState()
			    .getClass().getSimpleName();
		    imesStatemachine.checkTransitionRequest();
		    logger.fine("Transition requests checked.");
		    // If the State has changed print the new State
		    if (imesStatemachine.stateChanged) {
			logger.info("State-machine statechange from "
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
		    logger.fine("Calculating new control parameters...");
		    imesStatemachine.updateCtrlParam();
		    logger.fine("New control parameters calculated.");

		    /*
		     * Change the control mode settings of the robot and send a
		     * new Destination pose.
		     */
		    try {
			logger.fine("Apply new control parameters...");
			logger.finest("Setting control mode settings for mode "
				+ "and destination..."
				+ imesStatemachine.controlMode.toString());
			smartServoRuntime
				.changeControlModeSettings(imesStatemachine.controlMode);
			logger.finest("Waiting for smartServo to read new control data...");
			smartServoRuntime.waitForTransferred();

			smartServoRuntime
				.setDestination(imesStatemachine.cmdPose);
			logger.fine("New control parameters applied");
		    } catch (Exception e) {
			logger.log(Level.SEVERE,
				"Cannot change control mode settings or command a new pose. "
					+ "Resetting smart servo", e);
			logger.warning("Activating waitForPause during "
				+ "setDestination on smartServo interface");
			smartServoRuntime
				.activatewaitForPauseDuringSetDestination(true);
			// logger.fine("Resetting controller, ack errors and reinit "
			// + "smartServo...");
			// ServoMotionUtilities
			// .resetControllerAndKILLALLMOTIONS(imesLBR);
			// ServoMotionUtilities.acknowledgeError(imesLBR);
			// initSmartServo();
			// logger.fine("Resetting done.");
		    }

		    // Defining the acknowledgment String for Control Interface
		    imesStatemachine.setAckPacket();
		    logger.fine("Acknowledgement package set in statemachine.");

		    /*
		     * sleep for a specified time (according to the loops
		     * iteration time).
		     */
		    SleepUtil.cyclicSleep(startTimeStamp, 2, MS_TO_SLEEP);

		    // Overall timing end
		    timer.loopEnd();
		    if (imesStatemachine.End) {
			logger.info("State machine was stopped, ending main loop.");
			stateMachineRun = false;

		    }
		} catch (InterruptedException e) {

		    logger.log(Level.SEVERE, "Interruption during main loop."
			    + "All connections will be closed.", e);
		    mainLoopActive = false;
		}
		logger.fine("End main loop");
	    } // end while

	} finally {
	    // Print final infos.
	    printFinalInfos();
	    mainLoopActive = false;
	}

	logger.exiting(this.getClass().getName(), "run()");

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
	logger.entering(this.getClass().getName(),
		"onApplicationStateChanged()", state.toString());

	switch (state) {
	case STOPPING:
	    logger.fine("Stopping detected. Disabling main loop...");
	    stateMachineRun = false;
	case MOTIONPAUSING:
	    logger.fine("Motion pausing detected. "
		    + "Showing smartPad dialog for abortion.");
	    /*
	     * Check if user wants to abort the statemachine via the smartpad by
	     * an ui-dialog.
	     */
	    final String[] possibleAnswers = { "Yes", "No" };
	    final int answerIndex = this.getApplicationUI().displayModalDialog(
		    ApplicationDialogType.QUESTION, "Abort the state machine?",
		    possibleAnswers[0], possibleAnswers[1]);
	    logger.fine("SmartPad dialog returned with answer "
		    + possibleAnswers[answerIndex]);
	    if (answerIndex == 0) {
		logger.info("Aborting statemachine...");
		stateMachineRun = false;
		logger.fine("Waiting for run() to end...");
		while (mainLoopActive) {
		    ThreadUtil.milliSleep(MS_TO_SLEEP);
		}
		dispose();

	    } else {
		logger.info("Continuing statemachine...");
	    }
	default:
	    break;
	}

	logger.exiting(this.getClass().getName(),
		"onApplicationStateChanged(...)");

    }

    /**
     * Prints timing statistics and communication parameters.
     * 
     */
    private void printFinalInfos() {
	// Print the timing statistics
	logger.info("Displaying final states after loop "
		+ controlMode.getClass().getName());
	smartServoRuntime.setDetailedOutput(1);
	logger.info(timer.getOverallStatistics());

	if (timer.getMeanTimeMillis() > MS_TO_SLEEP
		+ MAXIMUM_TIMING_DEVIATION_MS) {
	    logger.info("Statistic Timing is unexpected slow, "
		    + "you should try to optimize TCP/IP Transfer");
	    logger.info("Under Windows, you should play with the registry, "
		    + "see the e.g. the RealtimePTP Class javaDoc for details");
	}
    }

    /**
     * Stops all running communication threads And stops the motion.
     */
    public final void dispose() {

	logger.entering(this.getClass().getName(), "dispose()");

	// Stop the motion
	final boolean motionStopped = smartServoRuntime.stopMotion();
	if (!motionStopped) {
	    logger.severe("Cannot stop motion of smartServoRuntime.");
	} else {
	    logger.fine("Motion stopped of smartServo.");
	}

	// Stop all threads.
	logger.fine("Interrupting all threads...");
	controlThread.interrupt();
	visualizationThread.interrupt();
	try {
	    logger.fine("Waiting for threads to join for " + JOIN_TIME_THREADS
		    + "ms...");
	    controlThread.join(JOIN_TIME_THREADS);
	    logger.fine("Control thread joined.");
	    visualizationThread.join(JOIN_TIME_THREADS);
	    logger.fine("visualization thread joined.");
	} catch (InterruptedException e) {
	    logger.log(Level.SEVERE,
		    "Waiting for the ending of the communication threads "
			    + "was interrupted.", e);
	}

	logger.info("State machine was disposed properly.");
	logger.exiting(this.getClass().getName(), "dispose()");
	logConfig.dispose();
	super.dispose();

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
