/*=========================================================================

  Program:   LWRStateMachineInterface
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher. Institute of Mechatronics System, Leibniz Universitaet Hannover. All rights reserved.

	See License.txt for more information
	
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

package de.uniHannover.imes.igtIf.communication.control;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.math.BigInteger;
import java.nio.ByteBuffer;

import OpenIGTLink.swig.ByteArr;
import OpenIGTLink.swig.IGTLheader;
import OpenIGTLink.swig.IGTLstring;
import OpenIGTLink.swig.igtl_header;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;
import de.uniHannover.imes.igtIf.communication.IGTLCommunicator;
import de.uniHannover.imes.igtIf.communication.IGTLMsg;
import de.uniHannover.imes.igtIf.logging.DummyLogger;
import de.uniHannover.imes.igtIf.util.FileSystemUtil;

/**
 * This Class for the Communication with a control system using the opnIGTLink
 * protocol is based on the igtlink4j class developed at the WPI. For Further
 * information see WPI website. The LWRStateMachineInterface Thread is operated
 * with a 20ms cycle. An Example of the use of this class for the communication
 * with a State Control Software e.g. 3D Slicer see imesStateApllication.
 * According of the data type of the OpenIGTMessage the data is packed and send
 * to the state control. Supported data types are "STATUS", "STRING" and
 * "TRANSFORM". Later on "POINT" will be supported.
 * 
 * @author Sebastian Tauscher
 * @see SimpleStateExample
 */
public class LWRStateMachineInterface extends Thread {

    /**
     * The port for the slicer-control-thread.
     */
    public static final int SLICER_CONTROL_COM_PORT = 49001;

    /** Default String encoding for the OpenIGTLink String data type. */
    private static final int DEFAULT_STRING_ENCODING = 3;

    /**
     * The maximum allowed connection error for udp / tcp communication.
     */
    private static final int MAX_ALLOWED_CONNECTION_ERR = 100;

    /**
     * Maximum allowed number of repitive received equal uids.
     */
    private static final int MAX_EQUAL_UIDS = 4;

    /** Number of elements of a rotational matrix. */
    private static final int SIZE_OF_ROTATION = 9;

    /** Number of elements of a translation vector. */
    private static final int SIZE_OF_TRANS = 3;

    /**
     * Default cycle time for this main loop in milliseconds.
     */
    private static final int CYCLE_TIME_DEFAULT = 20;

    /**
     * External swig library.
     */
    private static final String SWIG_DLL = "OpenIGTLinkLib/SWIGigtlutil.dll";

    /**
     * Relative dll path in jar.
     */
    private static final String SWIG_DLL_RELPATH = "OpenIGTLinkLib/";

    /**
     * Relative library path in project directory.
     */
    private static final String LIB_PATH_REL = File.separatorChar + "Libs"
	    + File.separatorChar + "SWIG" + File.separatorChar
	    + "SWIG_communication.jar";

    /**
     * Load SWIG igtlutil library (Default Library folder is
     * "..\OpenIGTLinkLib\swig\"
     */
    static {
	/*
	 * To load the correct swig library we need to extract the file. It is
	 * packed by sunrise workbench to a jar-archive. The dll-file can be
	 * found in another jar-archive.
	 */
	// Get path to jar.
	File ProjectParentDir = new File(System.getProperty("user.dir")
		+ File.separatorChar + "Git" + File.separatorChar);
	File[] dirs = ProjectParentDir.listFiles();
	File projectDir = dirs[0];
	File JarSource = new File(projectDir.getAbsolutePath() + LIB_PATH_REL);
	File JarDestination = new File(projectDir.getAbsolutePath()
		+ File.separatorChar + SWIG_DLL);
	System.out.println("Source File: " + JarSource.getAbsolutePath());
	System.out.println("Dest File: " + JarDestination.getAbsolutePath());
	try {
	    FileSystemUtil.extractFileFromJar(JarSource, JarDestination,
		    SWIG_DLL_RELPATH + SWIG_DLL);
	} catch (IOException e) {
	    e.printStackTrace();
	}
	System.load(JarDestination.getAbsolutePath());
    }

    /**
     * Acknowledgement OpenIGTLink Message working copy.
     */
    private String ackMsg;

    /**
     * Object handles IGTL communication.
     */
    private IGTLCommunicator communicator;

    /**
     * Provider for command data received via another openIGTL channel.
     */
    private CommunicationDataProvider comDataSink;

    /**
     * Time step for statistic timing of the State Machine Interface thread.
     */
    private OneTimeStep aStep;

    /**
     * Flag to indicate if the communication interface is running or not.
     */
    public boolean comRunning = false; // TODO design failure other threads
				       // access this field.

    /**
     * input stream of the socket communication.
     */
    private InputStream instr;

    /**
     * cycle time of the state control interface thread. Default value is 50 ms.
     */
    public int millisectoSleep = 50; // TODO design failure other threads access
				     // this field.

    /**
     * Statistic Timer for the State Machine Interface Thread.
     */
    public StatisticTimer SMtiming = new StatisticTimer(); // TODO design
							   // failure other
							   // threads access
							   // this field.

    /**
     * Logging mechanism object.
     */
    private ITaskLogger log;

    /**
     * cycle time of the state machine interface thread. Default value is 25 ms.
     */
    private int cycleTime;

    /**
     * Counter for connection errors.
     */
    private int connectionErrCounter;

    /**
     * Constructor, which initializes this thread as a daemon.
     * 
     * @param comDataProvider
     *            provider for command data received via another IGTL channel.
     */
    public LWRStateMachineInterface(
	    final CommunicationDataProvider comDataProvider) {
	init(comDataProvider, new DummyLogger(), CYCLE_TIME_DEFAULT);
    }

    /**
     * Constructor, which initializes this thread as a daemon.
     * 
     * @param comDataProvider
     *            provider for command data received via another IGTL channel.
     * 
     * @param cycleTimeMs
     *            the desired cycle time for the main loop in milliseconds.
     */
    public LWRStateMachineInterface(
	    final CommunicationDataProvider comDataProvider,
	    final int cycleTimeMs) {
	init(comDataProvider, new DummyLogger(), cycleTimeMs);
    }

    /**
     * Constructor, which initializes this thread as a daemon.
     * 
     * @param cycleTimeMs
     *            the desired cycle time for the main loop in milliseconds.
     * @param logger
     *            an external logger which collects the logging output of this
     *            class.
     */
    public LWRStateMachineInterface(
	    final CommunicationDataProvider comDataProvider,
	    final int cycleTimeMs, final ITaskLogger logger) {
	init(comDataProvider, logger, cycleTimeMs);
    }

    /**
     * Initializes this class according to the parameters which are set before
     * in the different constructors.
     * 
     * @param comDataProvider
     *            provider for command data received via another IGTL channel.
     * 
     * @param logger
     *            the task logger.
     * @param cycletime
     *            the desired cycle time in ms.
     */
    private void init(final CommunicationDataProvider comDataProvider,
	    final ITaskLogger logger, final int cycletime) {
	comDataSink = comDataProvider;
	log = logger;
	cycleTime = cycletime;
	setDaemon(true);
	communicator = new IGTLCommunicator(SLICER_CONTROL_COM_PORT, cycleTime);
	connectionErrCounter = 0;
    }

    // /**
    // * Starts the listening server on the defined port.
    // *
    // * @throws IOException
    // * when connection to the port fails.
    // */
    // private void connectServer() throws IOException {
    // stopServer();
    // try {
    // ServerSocketFactory serverSocketFactory = ServerSocketFactory
    // .getDefault();
    // openIGTServer = serverSocketFactory.createServerSocket(this.port);
    // openIGTServer.setReuseAddress(true);
    // log.info("State machine interface server Socket succesfully "
    // + "created (port " + this.port + ")");
    // } catch (IOException e) {
    // log.error("Could not Connect to port :" + this.port + ")", e);
    // throw e; // TODO exception concept.
    // }
    // }

    // /**
    // * Closes the connection to the server and the client.
    // */
    // public final void finalize() {
    // if (openIGTServer != null) {
    // try {
    // openIGTServer.close();
    // openIGTServer = null;
    // openIGTClient.close();
    // openIGTClient = null;
    // log.info("State machine interface server stopped");
    // } catch (IOException e) {
    // log.error(
    // "Could not disconnect from state machine interface server",
    // e);
    // e.printStackTrace(); // TODO exception concept.
    // }
    // }
    //
    // };

    /**
     * In this function a message from the Client Socket is received and
     * according to the OpenIGTLink datatype the recieved data is saved in the
     * member variable CMDmessage. If the data type is neither Transform nor
     * String an ErrorMessage is created. (Error Message not used yet...)
     * 
     * @throws IOException
     *             when the received data has the wrong datatype or is
     *             incomplete.
     */
    private void receiveMessage() throws IOException {

	IGTLMsg receivedMsg;
	byte[] headerBytes;
	byte[] bodyBytes;
	int numOfHeaderBytesRead;
	int numOfBodyBytesRead;
	int bodySize;

	do {
	    // Init header frame copy field.
	    headerBytes = new byte[IGTLheader.IGTL_HEADER_SIZE];

	    // Read header frame.
	    numOfHeaderBytesRead = instr.read(headerBytes);
	    if (numOfHeaderBytesRead < headerBytes.length) {
		throw new UnknownCommandException(
			"Unsufficent number of header bytes read");
	    }

	    // Extract size of body frame.
	    bodySize = IGTLCommunicator.extractBodySize(headerBytes);

	    // Init body frame copy field.
	    bodyBytes = new byte[bodySize];

	    // Read body bytes
	    numOfBodyBytesRead = instr.read(bodyBytes);
	    if (numOfBodyBytesRead < bodyBytes.length) {
		throw new UnknownCommandException(
			"Unsufficent number of body bytes read");
	    }

	    // Construct a openIGTL message from header and body.
	    receivedMsg = new IGTLMsg();
	    receivedMsg.init(headerBytes, bodyBytes);

	} while (!comDataSink.readNewCmdMessage(receivedMsg)); // checks the new
							       // message and
							       // saves it if it
							       // has new
							       // content.
    }

    /**
     * main/run method function of the State control Interface. In this function
     * the server is initialized and a packet handler is started. In a loop with
     * a cycle time of 20 ms the new Command String is received and the
     * Acknowledgment String send to the state control (e.g. 3d Slicer).
     **/
    public final void run() {

	// Initializing the Communication with Slicer
	try {
	    communicator.setup();
	    log.info("State machine interface client connected ( "
		    + communicator.getInetAddress() + ", "
		    + communicator.getPort() + ")");

	} catch (IOException e1) {
	    log.error("Initial set up of communicator failed.", e1);
	}

	comRunning = true; // TODO Tobi

	try {
	    if (!communicator.isClosed()) {
		sendIGTStringMessage(getAckMsg()
			+ comDataSink.getCurrentCmdPacket().getUid() + ";");
	    }
	} catch (Exception e1) {
	    log.error("StateMachineIF: Couldn't Send ACk data");
	}

	// Entering Loop for Communication - the loop is stopped if ControlRun
	// is set to false
	while (comRunning) {
	    // Starting Time
	    long startTimeStamp = (long) (System.nanoTime());

	    aStep = SMtiming.newTimeStep();
	    if (!communicator.isClosed()) {

		try {
		    // Receive Message from State Control
		    receiveMessage(); // TODO deal with Runtime exceptions
				      // properly
		    sendIGTStringMessage(getAckMsg()
			    + comDataSink.getCurrentCmdPacket().getUid() + ";");
		} catch (IOException e) {
		    connectionErrCounter++;
		}

	    } else { // If there is an connection error stop listening server
		     // and restart the server
		try {
		    communicator.restart();
		} catch (IOException e) {
		    log.error("Could not reestablish openIGTL connection", e);

		}
	    }

	    // Set the Module in Sleep mode for stability enhancement
	    try {
		StateMachineApplication.cyclicSleep(startTimeStamp, 2,
			millisectoSleep);
	    } catch (InterruptedException e) {
		// TODO exception concept.
		log.error("Thread Sleep failed!");
	    }

	    if (connectionErrCounter >= MAX_ALLOWED_CONNECTION_ERR) {
		log.error("Got " + connectionErrCounter
			+ " connection errors. Stopping communication.");
		break;
	    }

	    aStep.end();

	    /*
	     * analysis of timer statistics.
	     */
	    // TODO @Sebastian following section must be simplified
	    if (SMtiming.getMaxTimeMillis() >= 10 * millisectoSleep
		    || SMtiming.getMeanTimeMillis() >= 2 * millisectoSleep) {
		log.error("StateMachineIF: Attention! Bad communication quality "
			+ "robot changes state to Error!");
	    } else if ((SMtiming.getMaxTimeMillis() > 3.0 * millisectoSleep && SMtiming
		    .getMaxTimeMillis() < 10.0 * millisectoSleep)
		    || (SMtiming.getMeanTimeMillis() > millisectoSleep + 5 && SMtiming
			    .getMeanTimeMillis() < 2 * millisectoSleep)) {
		log.error("StateMachineIF: Warning bad communication quality!");
	    }
	} // end while

	// End all communication, when run() ends.
	try {
	    communicator.dispose();
	} catch (IOException e) {
	    log.error("Closing of the IGTL connection failed", e);
	}

    }

    /**
     * Sets the current acknowledgement message.
     * 
     * @param msg
     *            the message for ack.
     */
    public final void setAckMsg(final String msg) {
	synchronized (ackMsg) {
	    ackMsg = msg;
	}
    }

    /**
     * Getter for acknowledgement message.
     * 
     * @return the ack msg.
     */
    private String getAckMsg() {
	synchronized (ackMsg) {
	    return ackMsg;
	}

    }

    // /**
    // * Sends bytes.
    // *
    // * @param bytes
    // * the array to be send.
    // * @throws IOException
    // * when sending fails.
    // */
    // private synchronized void sendBytes(byte[] bytes) throws IOException {
    // outstr.write(bytes);
    // outstr.flush();
    // }

    /**
     * Sends an OpenIGTlink String message.
     * 
     * @param message
     *            the message to be send.
     * 
     */
    private void sendIGTStringMessage(final String message) {

	IGTLMsg currentMsg;

	byte[] bodyByte = new byte[message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE];
	byte[] headerByte = new byte[IGTLheader.IGTL_HEADER_SIZE];
	igtl_header header = new igtl_header();
	header.setVersion(IGTLheader.IGTL_HEADER_VERSION);
	header.setBody_size((BigInteger.valueOf(message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE)));
	header.setName("STRING");
	header.setDevice_name("ACK"); /* Device name */
	header.setTimestamp(BigInteger.valueOf(System.nanoTime()));
	ByteBuffer bodyBuffer = ByteBuffer.allocate(message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE);
	bodyBuffer.putShort((short) DEFAULT_STRING_ENCODING);
	bodyBuffer.putShort((short) message.length());
	bodyBuffer.put(message.getBytes());

	bodyByte = bodyBuffer.array();
	ByteArr bodyArray = new ByteArr(message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE);
	for (int i = 0; i < message.length()
		+ IGTLstring.IGTL_STRING_HEADER_SIZE; i++) {
	    bodyArray.setitem(i, bodyByte[i]);
	}
	ByteArr headerArray = ByteArr.frompointer(IGTLheader.PackHeader(header,
		bodyArray.cast()));
	for (int i = 0; i < IGTLheader.IGTL_HEADER_SIZE; i++) {
	    headerByte[i] = (byte) headerArray.getitem(i);
	}

	try {
	    currentMsg = new IGTLMsg();
	    currentMsg.init(headerByte, bodyByte);
	    communicator.sendMsg(currentMsg);
	} catch (IOException e) {
	    log.error("Sending of bytes failed.", e);
	    // TODO exception concept.
	}

    }

    // /**
    // * Closes the IGT server and client connection.
    // *
    // */
    // private void stopServer() {
    //
    // if (openIGTServer != null) {
    // try {
    // openIGTServer.close();
    // openIGTServer = null; // TODO besser reset function
    // openIGTClient.close();
    // openIGTClient = null; // TODO besser reset function
    // System.out.println("State machine interface server stopped");
    // } catch (IOException e) {
    // log.error("Cannot close server or client connection", e);
    // // TODO exception concept.
    // }
    // }
    // }

}
