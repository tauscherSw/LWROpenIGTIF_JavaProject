package de.uniHannover.imes.igtIf.application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.logging.Level;
import java.util.logging.Logger;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.geometricModel.physics.Inertia;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

import de.uniHannover.imes.igtIf.logging.LwrIgtlLogConfigurator;

/**
 * This class is used to command all robot parametrization like motions or
 * impedance behavior to the sunrise controller.
 *
 */
public final class RobotMotionCommander {

    // **************************Constants**********************/
    /**
     * Definition of the initial robot position before the state-machine starts
     * working.
     */
    private static final JointPosition INITIAL_ROBOT_POSE = new JointPosition(
	    0.0, Math.toRadians(30), 0., -Math.toRadians(60), 0.,
	    Math.toRadians(90), 0.);

    /**
     * The tool object describing the physical properties of the tool attached
     * to the robot's flange.
     */
    private static final Tool IMES_TOOL = new Tool("Imes Tool", new LoadData(
	    0.44, MatrixTransformation.ofTranslation(-5, 0, 50), Inertia.ZERO));

    /**
     * Definition of the tool-center-point.
     */
    private static final MatrixTransformation TOOL_TCP_OFFSET = MatrixTransformation
	    .ofTranslation(-40, 10, 207);

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

    /**
     * Acceleration of robot movements during state-machine execution. Value in
     * %.
     */
    private static final double ACC = 1;

    // **************************Components*********************/
    /**
     * Control mode for movements during state control.
     */
    private IMotionControlMode controlMode;

    /** The robot object for controlling robot movements. */
    private LBR robot;

    /** Singleton instance of the class. */
    private static RobotMotionCommander instance;

    /** Logger for this class. */
    private Logger logger = Logger
	    .getLogger(LwrIgtlLogConfigurator.LOGGERS_NAME);

    /** Application ui for displaying questions etc. on the smartPad. */
    private IApplicationUI ui;

    /** Sunrise specific interface to control the robot's movements. */
    private ISmartServoRuntime smartServoRuntime;

    /** Realtime motion, which is continously commanded. */
    private SmartServo realtimeMotion;

    /** Motion container of the realtime motion. */
    private IMotionContainer realtimeMotionContainer;

    // ***************************Constructor***********************/
    /**
     * Intentionally privatized --> Singleton.
     */
    private RobotMotionCommander() {

    }

    // ***************************Methods**************************/
    /**
     * Provides a singleton instance of this class.
     * 
     * @return an instance.
     */
    public static RobotMotionCommander getInstance() {
	if (null == instance) {
	    instance = new RobotMotionCommander();
	}
	return instance;
    }

    /**
     * Initialize this class with robot-corresponding objects of your sunrise
     * environment.
     * 
     * @param lwRobot
     *            the robot object.
     * @param robotAppUi
     *            the ui of the sunrise application.
     */
    public void init(final LBR lwRobot, final IApplicationUI robotAppUi) {
	if (null == lwRobot || null == robotAppUi) {
	    throw new NullPointerException("Argument is null");
	}
	ui = robotAppUi;
	robot = lwRobot;
    }

    /**
     * Initializes the whole robot environment. This includes resetting of the
     * controller, parametrization of the robot tool, moving to initial
     * position, validating load data and setting up the smart servo interface.
     */
    public void setup() {

	/* Reset Sunrise controller and ack possible errors. */
	ServoMotionUtilities.resetControllerAndKILLALLMOTIONS(robot);
	ServoMotionUtilities.acknowledgeError(robot);
	logger.finest("Resetted sunrise controller and acked all errors.");

	/* Initialize tool and it's attachment. */
	IMES_TOOL.addDefaultMotionFrame("TCP", TOOL_TCP_OFFSET);
	IMES_TOOL.attachTo(robot.getFlange());
	logger.finest("Tool attached to the robot object.");

	/*
	 * Check load data and then move to initial position. User interaction
	 * via the smartPad is needed therefore.
	 */
	logger.finest("Show SmartPad dialog about going to intial pose.");
	final int answerOnDialog = ui.displayModalDialog(
		ApplicationDialogType.WARNING,
		"Robot will move to initial joint position ("
			+ INITIAL_ROBOT_POSE.toString() + ") if prompted!.",
		"OK");
	logger.finest("SmartPad dialog returned.");
	if (answerOnDialog == 0) {
	    logger.info("Dialog prompted by user.");
	    IMES_TOOL.move(ptp(INITIAL_ROBOT_POSE));
	    logger.info("Robot moved to initial position.");
	} else {
	    logger.severe("SmartPad dialog cancelled.");
	    throw new IllegalStateException(
		    "Robot cannot move to intitial pose, "
			    + "because user dialog was cancelled.");
	}

	logger.info("Checking load data...");
	if (!SmartServo.validateForImpedanceMode(robot)) {
	    logger.severe("Validation of load data failed.");
	    throw new IllegalStateException("Load data is incorrect.");
	} else {
	    logger.info("Load data is validated succesfully.");
	}

	// Initialize smart servo interface
	initSmartServo();
	logger.finest("Smart servo and thread-exception-handler initialized.");

	logger.exiting(this.getClass().getName(), "setup()");
    }

    /**
     * Initializes the smart servo interface.
     */
    private void initSmartServo() {
	logger.entering(this.getClass().getName(), "initSmartServo()");

	// Initializing the SmartServo
	realtimeMotion = new SmartServo(robot.getCurrentJointPosition());
	realtimeMotion.useTrace(true);

	// Set the motion properties of all robot motions during state machine
	// execution.
	realtimeMotion.setJointAccelerationRel(VEL);
	realtimeMotion.setJointVelocityRel(ACC);

	/*
	 * Define and parameterize the control mode for the state machine
	 * execution.
	 */
	controlMode = new CartesianImpedanceControlMode();
	paramCartesianImpedanceMode(controlMode);
	logger.finest("Control mode parametrized as "
		+ controlMode.getClass().getSimpleName());

	logger.fine("Starting SmartServo Realtime Motion in "
		+ controlMode.getClass().getSimpleName());

	// Set the control mode as member of the realtime motion
	realtimeMotionContainer = IMES_TOOL.getDefaultMotionFrame()
		.moveAsync(realtimeMotion.setMode(controlMode));

	// Fetch the Runtime of the Motion part
	// NOTE: the Runtime will exist AFTER motion command was issued
	smartServoRuntime = realtimeMotion.getRuntime();
	smartServoRuntime.setMinimumTrajectoryExecutionTime(TRAJ_EXC_TIME);

	// Reading the current a couple of times for safety reasons
	smartServoRuntime.updateWithRealtimeSystem();

	// smartServoRuntime.updateWithRealtimeSystem(); //TODO @TOBI
	// ThreadUtil.milliSleep(MS_TO_SLEEP);

	logger.exiting(this.getClass().getName(), "initSmartServo()");
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
     * This method is intended for cyclical commanding of the smartServo
     * interface. It updates the system, sets the new control mode and the new
     * destination.
     * 
     * @param ctrlMode
     *            the new motion control mode.
     * @param destination
     *            the new destination
     */
    public void reconfigMotion(final IMotionControlMode ctrlMode,
	    final MatrixTransformation destination) {
	logger.entering(this.getClass().getName(), "reconfigMotion()");

	logger.fine("New command for smartServo: " + ctrlMode.toString() + " | "
		+ XyzAbcTransformation.of(destination).toString());

	/* Update with controller of LWR. */
	try {
	    smartServoRuntime.updateWithRealtimeSystem();
	    logger.fine("Smart-servo-runtime updated.");

	} catch (Exception e) {

	    logger.log(Level.WARNING,
		    "Failed to update the smart servo runtime.");
	}

	/* Apply new control parameters and set new destination. */
	try {

	    logger.fine("Apply new control parameters...");
	    logger.finest("Setting control mode settings for mode "
		    + "and destination..." + ctrlMode.toString());
	    smartServoRuntime.changeControlModeSettings(ctrlMode);
	    logger.finest("Waiting for smartServo to read new control data...");
	    // smartServoRuntime.waitForTransferred();

	    smartServoRuntime.setDestination(destination);
	    logger.fine("New control parameters applied");

	} catch (Exception e) {
	    logger.log(Level.SEVERE,
		    "Cannot change control mode settings or command a new pose. "
			    + "Resetting smart servo",
		    e);
	    logger.warning("Activating waitForPause during "
		    + "setDestination on smartServo interface");
	    smartServoRuntime.activatewaitForPauseDuringSetDestination(true);

	}

	logger.exiting(this.getClass().getName(), "reconfigMotion()");

    }

    /**
     * Stops all motions and disposes this class.
     */
    public void dispose() {
	logger.entering(this.getClass().getName(), "dispose()");

	realtimeMotionContainer.cancel();

	final boolean motionStopped = smartServoRuntime.stopMotion();

	if (!motionStopped) {
	    logger.severe("Cannot stop motion of smartServoRuntime.");
	} else {
	    logger.fine("Motion stopped of smartServo.");
	}

	logger.exiting(this.getClass().getName(), "dispose()");
    }

    /**
     * Getter for the robot tool.
     * 
     * @return the robot tool
     */
    public Tool getTool() {
	return IMES_TOOL;
    }

    /**
     * Gives information about the smartServo runtime.
     * 
     * @return information about the smartServo runtime as String.
     */
    public String getAdditionRuntimeInfos() {
	return smartServoRuntime.toString();
    }

}
