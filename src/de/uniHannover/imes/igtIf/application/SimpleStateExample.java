package de.uniHannover.imes.igtIf.application;

import java.util.concurrent.TimeUnit;
import de.uniHannover.imes.igtIf.interfaces.LWRStateMachineInterface;
import de.uniHannover.imes.igtIf.interfaces.LWRVisualizationInterface;
import de.uniHannover.imes.igtIf.stateMachine.LWRStatemachine;

import com.kuka.common.StatisticTimer;
import com.kuka.common.ThreadUtil;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
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
 *  <code>
 *  {@code
 *    //Flag to indicate if the Visualization is active or not
 *    boolean VisualON=false;	
 *   
 *   // To set the Visualization active at the start of the program just set the StartVisual flag of the lwrStatemachine Object imesStatemachine to true,
 *   // e.g.  imesStatemachine.StartVisual=true. Else this flag is set if the StateControl sends the Command "Visual;true;img/rob/jnt"
 *   //imesStatemachine.StartVisual=true;
 * 
 *   // Declaration of a LWRStateMachineInterface Object for the communication with a State Control using OpenIGTLink
 *   LWRStateMachineInterface SlicerControlIf = new LWRStateMachineInterface();
 * 	
 *  	//Declaration of a LWRVisualizationInterface Object for the communication with Visualization using OpenIGTLink e.g. 3D Slicer OpenIGTIF
 * 	LWRVisualizationInterface SlicerVisualIF = new LWRVisualizationInterface();	
 * 	
 * 	//Setting the port for the Control Interface supported ports are 49001 to 49005. Default Value is 49001
 *    SlicerControlIf.port =49001;
 *    
 *    //Setting the port for the Visualization Interface supported ports are 49001 to 49005. Default Value is 49002
 * 	SlicerVisualIF.port = 49002;
 * }
 * </code>
 * </pre>
 * 
 * After this the SmartServo Motion needs to be initialized (see SmartServo
 * Documentation) and the SlicerControl thread is started.
 * 
 * <pre>
 * {@code
 *  	//initializing and starting of the AliveThread for the Communication with the  State controller
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
 * 	//Setting the imes State machine member variables such as the control Mode
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
public class SimpleStateExample extends RoboticsAPIApplication {

    /** Robot object. */
    private LBR imesLBR;

    /** Tool mounted on the robots flange. */
    private PhysicalObject imesTool;

    /** Runtime for controlling servo motion. */
    private ISmartServoRuntime smartServoRuntime;
    /**
     * Object of the State machine class.
     * 
     * @see LWRStatemachine
     */
    private LWRStatemachine imesStatemachine = new LWRStatemachine();
    /**
     * number of loops to run with out any communication with the state control.
     */
    private static final int N_OF_RUNS = 1000;

    /**
     * Cyclic time of each loop of the main (state machine) thread.
     */
    private static final int CYCLE_TIME = 15;

    @Override
    /**
     * In this function the robot, tool etc are initialized.
     **/
    public void initialize() {
	System.out
		.println("Initializing Tool and Validate load for SmartServo.");
	// Locate the "first" Lightweight Robot in the system
	imesLBR = (LBR) ServoMotionUtilities.locateLBR(getContext());
	// FIXME: Set proper Weights or use the plugin feature
	// The Translation to the Tool Tip in mm
	final double[] translationOfTool = { -5, 30.7, 216 };
	// and the mass in kg
	final double mass = 0.6;

	// First rough guess of the Center of Mass
	final double[] centerOfMassInMillimeter = { 0, 0, 50 };

	imesTool = ServoMotionUtilities.createTool(imesLBR,
		"SimpleJointMotionSampleTool", translationOfTool, mass,
		centerOfMassInMillimeter);
	ServoMotionUtilities.validateCurrentLoadSetting(imesLBR);
	// Reset the Controller on the LBR completely
	// WARNING:: THIS WILL KILL ALL MOTIONS -
	// AND WILL DISTURB OTHER APPLICATIONS RUNNING ON THE VERY SAME
	// CONTROLLER
	// DONT CALL THIS ROUTINE, IF YOU ARE NOT ALONE ON THE SYSTEM
	ServoMotionUtilities.resetControllerAndKILLALLMOTIONS(imesLBR);
	// You may call instead
	// If you just like to acknowledge pending errors
	ServoMotionUtilities.acknowledgeError(imesLBR);

    }

    /**
     * Move to an initial Position WARNING: MAKE SHURE, THAT the pose is
     * collision free.
     */
    public final void moveToInitialPosition() {
	System.out.println("Move to initial start Position "
		+ "before the State machine Application starts");
	imesTool.move(ptp(0.0, Math.PI / 180 * 30., 0., -Math.PI / 180 * 60.,
		0., Math.PI / 180 * 90., 0.).setJointVelocityRel(0.2));
	/*
	 * Note: The Validation itself justifies, that in this very time
	 * instance, the load parameter setting was sufficient. This does not
	 * mean by far, that the parameter setting is valid in the sequel or
	 * lifetime of this program
	 */
	if (SmartServo.validateForImpedanceMode(imesLBR) != true) {
	    System.out.println("Validation of Torque Model failed "
		    + "- correct your mass property settings");
	    System.out.println("RealtimePTP will be available for position "
		    + "controlled mode only, until validation is performed");
	}
    }

    /**
     * In this function the communication with the robot via RealTimePTP, the
     * communication with the Visualization and Control Software (e.g. 3D
     * Slicer, Matlab) and the State machine it self are operated.
     * 
     */

    public void runRealtimeMotion(IMotionControlMode controlMode) {

	String lastPrintedError = "";
	String errorMsg = "";
	// Flag to indicate if the Visualization is active or not
	boolean visualOnFlag = false;

	// To set the Visualization active at the start of the program just set
	// the StartVisual flag of the lwrStatemachine Object imesStatemachine
	// to true,
	// e.g. imesStatemachine.StartVisual=true. Else this flag is set if the
	// StateControl sends the Command "Visual;true;img/rob/jnt"
	// imesStatemachine.StartVisual=true;

	// Declaration of a LWRStateMachineInterface Object for the
	// communication with a State Control using OpenIGTLink
	LWRStateMachineInterface slicerControlIf = new LWRStateMachineInterface();
	slicerControlIf.setPriority(6);
	// Declaration of a LWRVisualizationInterface Object for the
	// communication with Visualization using OpenIGTLink e.g. 3D Slicer
	// OpenIGTIF
	LWRVisualizationInterface slicerVisualIf = new LWRVisualizationInterface();
	slicerVisualIf.setPriority(4);
	// Timer VisualTimer = new Timer(); // Instantiate Timer Object
	// Timer StateControlTimer = new Timer(); // Instantiate Timer Object

	// Setting the port for the Control Interface supported ports are 49001
	// to 49005. Default Value is 49001
	slicerControlIf.port = 49001;
	// Setting the port for the Visualization Interface supported ports are
	// 49001 to 49005. Default Value is 49002
	slicerVisualIf.port = 49002;

	boolean stateMachineRun = true;
	int i = 0;

	long curTime = 0;
	// int curTime_nanos=0;
	long startTimeStamp = System.nanoTime();
	// int startTimeStamp_nanos = 0;

	JointPosition initialPosition = new JointPosition(
		imesLBR.getCurrentJointPosition());

	// Initializing the SmartServo
	SmartServo aRealtimeMotion = new SmartServo(initialPosition);
	// aRealtimeMotion.useTrace(true);
	aRealtimeMotion.setMinimumSynchronizationTime(5e-3);
	// Set the motion properties to 10% of the systems abilities
	aRealtimeMotion.setJointAccelerationRel(0.3);
	aRealtimeMotion.setJointVelocityRel(0.2);

	System.out.println("Starting SmartServo Realtime Motion in "
		+ controlMode.getClass().getName());

	// Set the control mode as member of the realtime motion
	imesTool.getDefaultMotionFrame().moveAsync(
		aRealtimeMotion.setMode(controlMode));

	// Fetch the Runtime of the Motion part
	// NOTE: the Runtime will exist AFTER motion command was issued
	smartServoRuntime = aRealtimeMotion.getRuntime();
	smartServoRuntime.setMinimumSynchronizationTime(5e-3);

	// SmartServoRuntime.setMinimumSynchronizationTime(3);

	// For Roundtrip time measurement...

	System.out.println("Starting Thread for state control communication ");
	long curTimeMillis = 0;
	// initializing and starting of the AliveThread for the Communication
	// with the State controller
	// SlicerControlIf.start();

	try {

	    // Reading the current a couple of times for safety reasons
	    smartServoRuntime.updateWithRealtimeSystem();
	    ThreadUtil.milliSleep(CYCLE_TIME);
	    smartServoRuntime.updateWithRealtimeSystem();
	    ThreadUtil.milliSleep(CYCLE_TIME);

	    // Setting the imes State machine member variables such as the
	    // control Mode
	    imesStatemachine.curPose = MatrixTransformation
		    .of(smartServoRuntime.getCartFrmMsrOnController());
	    imesStatemachine.controlMode = controlMode;
	    imesStatemachine.cmdPose = MatrixTransformation
		    .of(smartServoRuntime.getCartFrmMsrOnController());

	    // Initialize some of the Visualization Interface member variables
	    slicerVisualIf.jntPose_StateM = smartServoRuntime
		    .getAxisQMsrOnController();
	    slicerVisualIf.cartPose_StateM = imesStatemachine.curPose;

	    // Know entering the main loop. In this loop the Command from the
	    // State Control is read, interpreted,
	    // the new parameters calculated and sent to the Robot via
	    // SmartServo and the ACknowlegment String is send back to the State
	    // Control.
	    // Therefore, the LWRStatemachine, the LWRStateMachineInterface and
	    // the LWRVisualization objects are used
	    StatisticTimer timing = new StatisticTimer();
	    while (stateMachineRun && i < N_OF_RUNS) {
		// loop Starting time for statistics
		startTimeStamp = (long) (System.nanoTime());

		// Timing - draw one step
		OneTimeStep aStep = timing.newTimeStep();

		// If flags are set and VisualIF is not running yet start the
		// Visual Thread
		if (imesStatemachine.StartVisual && visualOnFlag == false) {
		    // System.out.println("Starting Visualization Interface thread");
		    // Initialize the necessary member variables first
		    slicerVisualIf.jntPose_StateM = initialPosition;
		    slicerVisualIf.cartPose_StateM = imesStatemachine.curPose;
		    System.out.println("Setting datatype");
		    if (imesStatemachine.currentVisualIFDatatype == 1) {
			slicerVisualIf.datatype = LWRVisualizationInterface.VisualIFDatatypes.IMAGESPACE;
		    } else if (imesStatemachine.currentVisualIFDatatype == 2) {
			slicerVisualIf.datatype = LWRVisualizationInterface.VisualIFDatatypes.ROBOTBASE;
		    } else if (imesStatemachine.currentVisualIFDatatype == 3) {
			slicerVisualIf.datatype = LWRVisualizationInterface.VisualIFDatatypes.JOINTSPACE;

		    }
		    if (imesStatemachine.TransformRecieved) {
			slicerVisualIf.trafo_Imagespace_StateM = imesStatemachine.TransformRobotImage;
		    }
		    slicerVisualIf.visualIfActivated = true;
		    slicerVisualIf.start();
		    // Start the Visualization thread
		    // SlicerVisualIF.start();
		    visualOnFlag = true;
		} else if (imesStatemachine.StartVisual && visualOnFlag
			&& !slicerVisualIf.visualIfActivated) {// if Viuslaization
							  // interface is
							  // started, not active
							  // but is set active
		    // Change VisualActive to true. Thereby, the pose is send to
		    // the visualization
		    slicerVisualIf.visualIfActivated = true;

		} else if (!imesStatemachine.StartVisual && /*
							     * SlicerVisualIF.
							     * isAlive() &&
							     */
		slicerVisualIf.visualIfActivated) {
		    /*
		     * if the visualization is running and the the Start Visual
		     * flag is false and the interface is still active Set the
		     * VisualACtive flag to false - thereby, no more data is
		     * send to the visualization
		     */

		    slicerVisualIf.visualIfActivated = false;

		}

		// If SlicerControl Interface Thread is running...
		if (slicerControlIf.comIfActive) {

		    i = 0;
		    // Try to read new command String from SlicerControl (Alive)
		    // Thread
		    try {
			slicerControlIf.ctrlSema.tryAcquire(1,
				TimeUnit.MILLISECONDS);
			imesStatemachine.CmdIGTmessage = slicerControlIf.cmdStateMsg;
			imesStatemachine.UID = slicerControlIf.uId;
			slicerControlIf.ctrlSema.release();

		    } catch (InterruptedException e) {
			errorMsg = "Couldn't acquire Semaphore!!";

			if (!errorMsg.equals(lastPrintedError)) {
			    System.out.println(errorMsg);
			    lastPrintedError = errorMsg;
			}

		    }
		} else { // if it is not to Error handling
		    imesStatemachine.ErrorCode = 2;
		    errorMsg = "Slicer Control Interface not Alive...";

		    if (!errorMsg.equals(lastPrintedError)) {
			System.out.println(errorMsg);
			lastPrintedError = errorMsg;
		    }
		    i++;
		}
		// Check if there is a Transition Request and in that case
		// Change the state and interpret the command parameters by
		// calling the function InterpretCommandString of the Current
		// State
		imesStatemachine.CheckTransitionRequest();

		// If the State has changed print the new State
		if (imesStatemachine.InitFlag) {
		    System.out.println("Robot State has Changed to:"
			    + imesStatemachine.RobotState.name());

		}
		// Print Error messages if there where any Errors
		imesStatemachine.ErrorHandler();

		// Calculating the new control Param and Change the parameters
		imesStatemachine.CalcControlParam();

		// Change the control mode settings of the robot and send a new
		// Destination pose
		try {
		    smartServoRuntime
			    .changeControlModeSettings(imesStatemachine.controlMode);
		    smartServoRuntime.setDestination(imesStatemachine.cmdPose);
		} catch (Exception e) {
		    errorMsg = "Error: Failed to change Realtime Settings!!";
		    if (!errorMsg.equals(lastPrintedError)) {
			System.out.println(errorMsg);
			lastPrintedError = errorMsg;
		    }
		}

		// Defining the Acknowledgement String for Control Interface
		imesStatemachine.SetACKPacket();

		if (slicerControlIf.comIfActive) {
		    try {
			slicerControlIf.ctrlSema.tryAcquire(1,
				TimeUnit.MILLISECONDS);
			// try to update the ACK String for the ControlIF Thread
			slicerControlIf.ackStateMsg = imesStatemachine.AckIGTmessage;
			slicerControlIf.ctrlSema.release();
		    } catch (InterruptedException e) {
			errorMsg = "Error: Couldn't Acquire ControlIF Semaphore!!";
			if (!errorMsg.equals(lastPrintedError)) {
			    // System.out.println(ErrorMessage);
			    lastPrintedError = errorMsg;
			}
		    }
		}
		// if the Visualization Interface is active sending the Current
		// Position to the Visualization
		if (/* SlicerVisualIF.isAlive()&& */slicerVisualIf.visualIfRunning) {
		    try {
			slicerVisualIf.visualSema.tryAcquire(1,
				TimeUnit.MILLISECONDS);
			if (imesStatemachine.currentVisualIFDatatype == 1) {
			    slicerVisualIf.datatype = LWRVisualizationInterface.VisualIFDatatypes.IMAGESPACE;
			    slicerVisualIf.cartPose_StateM = imesStatemachine.curPose;
			    if (imesStatemachine.TransformRecieved) {
				slicerVisualIf.trafo_Imagespace_StateM = imesStatemachine.TransformRobotImage;
			    }
			} else if (imesStatemachine.currentVisualIFDatatype == 2) {
			    slicerVisualIf.datatype = LWRVisualizationInterface.VisualIFDatatypes.ROBOTBASE;
			    slicerVisualIf.cartPose_StateM = imesStatemachine.curPose;
			} else if (imesStatemachine.currentVisualIFDatatype == 3) {
			    slicerVisualIf.datatype = LWRVisualizationInterface.VisualIFDatatypes.JOINTSPACE;
			    slicerVisualIf.jntPose_StateM = imesStatemachine.curJntPose;
			}
			slicerVisualIf.visualSema.release();
		    } catch (InterruptedException e) {
			errorMsg = "Error: Couldn't acquire VisualIF Semaphore!!";
			if (!errorMsg.equals(lastPrintedError)) {
			    // System.out.println(ErrorMessage);
			    lastPrintedError = errorMsg;
			}
		    }
		}

		// Update with Realtime System (LWR)
		try {
		    smartServoRuntime.updateWithRealtimeSystem();
		    // Get the measured position in cartesian pose
		    imesStatemachine.curPose = MatrixTransformation
			    .of(smartServoRuntime.getCartFrmMsrOnController());
		    // command pose equal to currently measured Pose as default
		    // value
		    imesStatemachine.cmdPose = imesStatemachine.curPose;
		    // and the measured joint angles
		    imesStatemachine.curJntPose = smartServoRuntime
			    .getAxisQMsrOnController();
		} catch (Exception e) {
		    errorMsg = "Error: Failed Update with RealtimeSystem!!";
		    if (!errorMsg.equals(lastPrintedError)) {
			// System.out.println(ErrorMessage);
			lastPrintedError = errorMsg;
		    }

		}

		// Set the Module in Sleep mode for stability enhancement
		// i++;
		curTime = (long) ((System.nanoTime() - startTimeStamp));
		curTimeMillis = (long) curTime / 1000000;
		// curTime_nanos = (int) (curTime%1000000);
		// if(i%100 ==0){
		// System.out.println(" CurTime : " + curTime +" Nanos: "
		// +curTime_nanos);
		// }
		// while(millisectoSleep*1000000 -
		// (System.nanoTime()-startTimeStamp) > 10000){
		// Thread.yield();
		// }
		if (curTimeMillis < CYCLE_TIME) {
		    // TimeUnit.MILLISECONDS.sleep((millisectoSleep -
		    // curTime_millis));
		    // ThreadUtil.milliSleep((long) Math.floor((millisectoSleep
		    // - curTime_millis)));
		    // ThreadUtil.nanoSleep(millisectoSleep*1000000-curTime);
		    // Thread.sleep(millisectoSleep-1-curTime_millis,
		    // 999999-curTime_nanos);

		}

		// Overall timing end
		aStep.end();
		if (imesStatemachine.End || imesStatemachine.Quit
			|| imesStatemachine.Shutdown) {
		    stateMachineRun = false;

		}

	    } // end while

	    slicerControlIf.comIfActive = false;
	    // StateControlTimer.cancel();
	    slicerVisualIf.visualIfRunning = false;
	    // VisualTimer.cancel();
	    // Print the timing statistics
	    System.out
		    .println("Statistic Timing of Statemachine interface thread "
			    + slicerControlIf.timerVisualIf);
	    System.out.println("UID miss: " + slicerControlIf.uIdMissed);
	    System.out
		    .println("Statistic Timing of Visualisation interface thread "
			    + slicerVisualIf.visualIfTimer);
	    System.out.println("Statistic Timing of Statemachine Mean:"
		    + timing);
	    ThreadUtil.milliSleep((long) (4000));

	    // /////////////////////////////////////////////////
	    // Do or die: print statistics and parameters of the motion
	    System.out.println("Displaying final states after loop "
		    + controlMode.getClass().getName());
	    smartServoRuntime.setDetailedOutput(1);
	    // Stop the motion
	    smartServoRuntime.stopMotion();

	    if (timing.getMeanTimeMillis() > CYCLE_TIME + 5) {
		System.out.println("Statistic Timing is unexpected slow, "
			+ "you should try to optimize TCP/IP Transfer");
		System.out
			.println("Under Windows, you should play with the registry, "
				+ "see the e.g. the RealtimePTP Class javaDoc for details");
	    }
	    if (imesStatemachine.Shutdown) {
		// moveToInitialPosition();
	    }
	} catch (Exception e) {
	    System.out.println(e);
	    e.printStackTrace();
	    slicerControlIf.comIfActive = false;
	    // StateControlTimer.cancel();
	    slicerVisualIf.visualIfRunning = false;
	    // VisualTimer.cancel();
	}
	// Stopping the Control Interface thread and the viusalization thread

	ThreadUtil.milliSleep((long) (1000));

    }

    /**
     * Sets up an cartesian impedance control mode object. It is initialized
     * with hard-coded parameters.
     * 
     * @return the cartesion impedance control mode object.
     */
    protected final CartesianImpedanceControlMode createCartImp() {
	CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
	cartImp.parametrize(CartDOF.TRANSL).setStiffness(5000.);
	cartImp.parametrize(CartDOF.ROT).setStiffness(300.);
	cartImp.setNullSpaceStiffness(5000.);
	// For your own safety, shrink the motion abilities to useful limits
	cartImp.parametrize(CartDOF.TRANSL).setMaxPathDeviation(500.);
	cartImp.parametrize(CartDOF.ROT).setMaxPathDeviation(30.);
	// cartImp.parametrize(CartDOF.TRANSL).setMaxCartesianVelocity(1000);
	// cartImp.parametrize(CartDOF.ROT).setMaxCartesianVelocity(20);

	{
	    // optionally shrink the joint specific limits
	    // use the jointMaxDelta feature
	    // Watchout: This feature is useful, only if the nullSpaceStiffness
	    // is sufficient high, so that the joints will follow the motion
	    // within the limits
	    double[] maxJointDeltas = { 15, 15, 15, 15, 15, 15, 15 };
	    // cartImp.setMaxJointSpeed(maxJointDeltas);
	    // cartImp.setm.setMaxJointDeltas(maxJointDeltas);
	}
	return cartImp;
    }

    /**
     * Auto-generated method stub. Do not modify the contents of this method.
     * 
     * @param args
     *            unused string array
     */
    public static void main(final String[] args) {
	SimpleStateExample app = new SimpleStateExample();
	app.runApplication();
    }

    @Override
    public final void run() {

	// Initiliaze "instanz" of the RealtimePTP

	// Cartesian impedance sample

	moveToInitialPosition();

	// Initialize Cartesian impedance mode
	CartesianImpedanceControlMode cartImp = createCartImp();
	// run the realtime motion, as before done in the SimpleJointMotion
	// sample.
	// the only difference - pass on the InteractionControlStrategy...
	runRealtimeMotion(cartImp);
    }
}
