package deprecated;
///*=========================================================================
//
//  Program:   LWRStateMachineInterface
//  Language:  java
//  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT
//
//  Copyright (c) Sebastian Tauscher. Institute of Mechatronics System, Leibniz Universitaet Hannover. All rights reserved.
//
//	See License.txt for more information
//	
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//	CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//	EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//	PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//	NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//	=========================================================================*/
//
//package de.uniHannover.imes.igtIf.communication.control;
//
//import java.io.File;
//import java.io.IOException;
//import java.io.InputStream;
//import java.math.BigInteger;
//import java.nio.ByteBuffer;
//
//import OpenIGTLink.swig.ByteArr;
//import OpenIGTLink.swig.IGTLheader;
//import OpenIGTLink.swig.IGTLstring;
//import OpenIGTLink.swig.igtl_header;
//
//import com.kuka.common.StatisticTimer;
//import com.kuka.common.StatisticTimer.OneTimeStep;
//import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;
//
//import de.uniHannover.imes.igtIf.application.StateMachineApplication;
//import de.uniHannover.imes.igtIf.communication.layer.IGTLMsg;
//import de.uniHannover.imes.igtIf.communication.layer.IIGTLCommunicator;
//import de.uniHannover.imes.igtIf.communication.layer.IOpenIGTLMsg;
//import de.uniHannover.imes.igtIf.communication.layer.IIGTLCommunicator.Channel;
//import de.uniHannover.imes.igtIf.logging.DummyLogger;
//import de.uniHannover.imes.igtIf.util.FileSystemUtil;
//
///**
// * This Class for the Communication with a control system using the opnIGTLink
// * protocol is based on the igtlink4j class developed at the WPI. For Further
// * information see WPI website. The LWRStateMachineInterface Thread is operated
// * with a 20ms cycle. An Example of the use of this class for the communication
// * with a State Control Software e.g. 3D Slicer see imesStateApllication.
// * According of the data type of the OpenIGTMessage the data is packed and send
// * to the state control. Supported data types are "STATUS", "STRING" and
// * "TRANSFORM". Later on "POINT" will be supported.
// * 
// * @author Sebastian Tauscher
// * @see SimpleStateExample
// */
//public class LWRStateMachineInterface extends Thread {
//
//    /** Default String encoding for the OpenIGTLink String data type. */
//    private static final int DEFAULT_STRING_ENCODING = 3;
//
//    /**
//     * The maximum allowed connection error for udp / tcp communication.
//     */
//    private static final int MAX_ALLOWED_CONNECTION_ERR = 100;
//
//    /**
//     * Maximum allowed number of repitive received equal uids.
//     */
//    private static final int MAX_EQUAL_UIDS = 4;
//
//    /** Number of elements of a rotational matrix. */
//    private static final int SIZE_OF_ROTATION = 9;
//
//    /** Number of elements of a translation vector. */
//    private static final int SIZE_OF_TRANS = 3;
//
//    /**
//     * Default cycle time for this main loop in milliseconds.
//     */
//    private static final int CYCLE_TIME_DEFAULT = 20;
//
//    /**
//     * External swig library.
//     */
//    private static final String SWIG_DLL = "SWIGigtlutil.dll";
//
//    /**
//     * Relative dll path in jar.
//     */
//    private static final String SWIG_DLL_RELPATH = "OpenIGTLinkLib/";
//
//    /**
//     * Relative library path in project directory.
//     */
//    private static final String LIB_PATH_REL = File.separatorChar + "Libs"
//	    + File.separatorChar + "SWIG" + File.separatorChar
//	    + "SWIG_communication.jar";
//
//    /**
//     * Load SWIG igtlutil library (Default Library folder is
//     * "..\OpenIGTLinkLib\swig\"
//     */
//    static {
//	/*
//	 * To load the correct swig library we need to extract the file. It is
//	 * packed by sunrise workbench to a jar-archive. The dll-file can be
//	 * found in another jar-archive.
//	 */
//	// Get path to jar
//	File projParentDir = new File(System.getProperty("user.dir")
//		+ File.separatorChar + "Git" + File.separatorChar);
//	File[] dirs = projParentDir.listFiles();
//	File projectDir = dirs[0];
//	File jarSrc = new File(projectDir.getAbsolutePath() + LIB_PATH_REL);
//	File jarDest = new File(projectDir.getAbsolutePath()
//		+ File.separatorChar + SWIG_DLL);
//	try {
//	    FileSystemUtil.extractFileFromJar(jarSrc, jarDest, SWIG_DLL_RELPATH
//		    + SWIG_DLL);
//	} catch (IOException e) {
//	    e.printStackTrace();
//	}
//	System.load(jarDest.getAbsolutePath());
//    }
//
//    /**
//     * Acknowledgement OpenIGTLink Message working copy.
//     */
//    private String ackMsg;
//
//    /**
//     * Object handles IGTL communication.
//     */
//    private IIGTLCommunicator communicator;
//
//    /**
//     * Provider for command data received via another openIGTL channel.
//     */
//    private CommunicationDataProvider comDataSink;
//
//    /**
//     * Time step for statistic timing of the State Machine Interface thread.
//     */
//    private OneTimeStep aStep;
//
//    /**
//     * Flag to indicate if the communication interface is running or not.
//     */
//    public boolean comRunning = false; // TODO design failure other threads
//    // access this field.
//
//    /**
//     * input stream of the socket communication.
//     */
//    private InputStream instr;
//
//    /**
//     * cycle time of the state control interface thread. Default value is 50 ms.
//     */
//    public int millisectoSleep = 50; // TODO design failure other threads access
//    // this field.
//
//    /**
//     * Statistic Timer for the State Machine Interface Thread.
//     */
//    public StatisticTimer SMtiming = new StatisticTimer(); // TODO design
//    // failure other
//    // threads access
//    // this field.
//
//    /**
//     * Logging mechanism object.
//     */
//    private ITaskLogger log;
//
//    /**
//     * cycle time of the state machine interface thread. Default value is 25 ms.
//     */
//    private int cycleTime;
//
//    /**
//     * Counter for connection errors.
//     */
//    private int connectionErrCounter;
//
//    /**
//     * Constructor, which initializes this thread as a daemon.
//     * 
//     * @param comHandler
//     *            a handler which gains access to IGTL communication.
//     * @param comDataProvider
//     *            provider for command data received via another IGTL channel.
//     * @throws IOException
//     *             when setup of communication fails.
//     */
//    public LWRStateMachineInterface(final IIGTLCommunicator comHandler,
//	    final CommunicationDataProvider comDataProvider) throws IOException {
//	new LWRStateMachineInterface(comHandler, comDataProvider,
//		CYCLE_TIME_DEFAULT, new DummyLogger());
//    }
//
//    /**
//     * Constructor, which initializes this thread as a daemon. *
//     * 
//     * @param comHandler
//     *            a handler which gains access to IGTL communication.
//     * @param comDataProvider
//     *            provider for command data received via another IGTL channel.
//     * 
//     * @param cycleTimeMs
//     *            the desired cycle time for the main loop in milliseconds.
//     * @param logger
//     *            an external logger which collects the logging output of this
//     *            class.
//     * @throws IOException
//     *             when setup of communication fails.
//     */
//    public LWRStateMachineInterface(final IIGTLCommunicator comHandler,
//	    final CommunicationDataProvider comDataProvider,
//	    final int cycleTimeMs, final ITaskLogger logger) throws IOException {
//	communicator = comHandler;
//	comDataSink = comDataProvider;
//	log = logger;
//	cycleTime = cycleTimeMs;
//	connectionErrCounter = 0;
//    }
//
//    /**
//     * In this function a message from the Client Socket is received and
//     * according to the OpenIGTLink datatype the recieved data is saved in the
//     * member variable CMDmessage. If the data type is neither Transform nor
//     * String an ErrorMessage is created. (Error Message not used yet...)
//     * 
//     */
//    private void receiveMessage() {
//
//	IOpenIGTLMsg receivedMsg = null;
//
//	do {
//	    receivedMsg = communicator.receiveMsg();
//
//	} while (!comDataSink.readNewCmdMessage(receivedMsg)); // checks the new
//	// message and
//	// saves it if it
//	// has new
//	// content.
//    }
//
//    /**
//     * main/run method function of the State control Interface. In this function
//     * the server is initialized and a packet handler is started. In a loop with
//     * a cycle time of 20 ms the new Command String is received and the
//     * Acknowledgment String send to the state control (e.g. 3d Slicer).
//     **/
//    public final void run() {
//
//	comRunning = true; // TODO Tobi
//
//	// Entering Loop for Communication - the loop is stopped if ControlRun
//	// is set to false
//	log.info(this.getClass().getSimpleName() + " enters the main loop");
//	long currentUid = 0;
//	while (comRunning) {
//
//	    log.info(this.getClass().getSimpleName() + " begins its main loop");
//	    // Starting Time
//	    long startTimeStamp = (long) (System.nanoTime());
//
//	    aStep = SMtiming.newTimeStep();
//
//	    // Receive Message from State Control
//	    receiveMessage(); // TODO deal with Runtime exceptions
//	    // properly
//	    log.info(this.getClass().getSimpleName() + " received a message.");
//	    currentUid = comDataSink.getCurrentCmdPacket().getUid();
//	    communicator.sendIGTStringMessage(getAckMsg() + currentUid + ";");
//	    log.info(this.getClass().getSimpleName()
//		    + " sends an acknowledgement for msg with uid "
//		    + currentUid + ".");
//
//	    // Set the Module in Sleep mode for stability enhancement
//	    try {
//		StateMachineApplication.cyclicSleep(startTimeStamp, 2,
//			millisectoSleep);
//	    } catch (InterruptedException e) {
//		// TODO exception concept.
//		log.error(this.getClass().getSimpleName() + " sleep failed!!");
//	    }
//
//	    if (connectionErrCounter >= MAX_ALLOWED_CONNECTION_ERR) {
//		log.error("Got " + connectionErrCounter
//			+ " connection errors. Stopping communication.");
//		break;
//	    }
//
//	    aStep.end();
//
//	    /*
//	     * analysis of timer statistics.
//	     */
//	    // TODO @Sebastian following section must be simplified
//	    if (SMtiming.getMaxTimeMillis() >= 10 * millisectoSleep
//		    || SMtiming.getMeanTimeMillis() >= 2 * millisectoSleep) {
//		log.error("StateMachineIF: Attention! Bad communication quality "
//			+ "robot changes state to Error!");
//	    } else if ((SMtiming.getMaxTimeMillis() > 3.0 * millisectoSleep && SMtiming
//		    .getMaxTimeMillis() < 10.0 * millisectoSleep)
//		    || (SMtiming.getMeanTimeMillis() > millisectoSleep + 5 && SMtiming
//			    .getMeanTimeMillis() < 2 * millisectoSleep)) {
//		log.error("StateMachineIF: Warning bad communication quality!");
//	    }
//	    log.info(this.getName() + " ends its main loop");
//	} // end while
//
//    }
//
//    /**
//     * Sets the current acknowledgement message.
//     * 
//     * @param msg
//     *            the message for ack.
//     */
//    public final void setAckMsg(final String msg) {
//	synchronized (ackMsg) {
//	    ackMsg = msg;
//	}
//    }
//
//    /**
//     * Getter for acknowledgement message.
//     * 
//     * @return the ack msg.
//     */
//    private String getAckMsg() {
//	synchronized (ackMsg) {
//	    return ackMsg;
//	}
//
//    }
//
//       // /**
//    // * Closes the IGT server and client connection.
//    // *
//    // */
//    // private void stopServer() {
//    //
//    // if (openIGTServer != null) {
//    // try {
//    // openIGTServer.close();
//    // openIGTServer = null; // TODO besser reset function
//    // openIGTClient.close();
//    // openIGTClient = null; // TODO besser reset function
//    // System.out.println("State machine interface server stopped");
//    // } catch (IOException e) {
//    // log.error("Cannot close server or client connection", e);
//    // // TODO exception concept.
//    // }
//    // }
//    // }
//
//}
