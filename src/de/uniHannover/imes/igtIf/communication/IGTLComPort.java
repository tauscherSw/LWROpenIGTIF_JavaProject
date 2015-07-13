/*=========================================================================

  Program:   IGTLComPort
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

package de.uniHannover.imes.igtIf.communication;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.math.BigInteger;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.net.ServerSocketFactory;

import de.uniHannover.imes.igtIf.logging.LwrIgtlLogConfigurator;
import OpenIGTLink.swig.ByteArr;
import OpenIGTLink.swig.IGTLheader;
import OpenIGTLink.swig.IGTLstring;
import OpenIGTLink.swig.IGTLtransform;
import OpenIGTLink.swig.igtl_header;

/**
 * This class provides basic methods for sending and receiving messages to and
 * from via IGTLink.
 */
public final class IGTLComPort {

    // **************************Constants**********************/
    /** Default String encoding for the OpenIGTLink String data type. */
    private static final int DEFAULT_STRING_ENCODING = 3;

    /** Maximum supported port. */
    private static final int MAXIMUM_PORT_NUMBER = 49005;

    /** Minimum supported port. */
    private static final int MINIMUM_PORT_NUMBER = 49001;

    /** Size of the bodysize field in the IGT communication header. */
    private static final int SIZE_OF_BODYSIZE_FIELD = 8;
    // TODO @Sebastian is there a constant for size of bodysize field in SWIG?

    /**
     * Begin of the bodysize field in the IGT communication header counted in
     * bytes.
     */
    private static final int BEGIN_POS_BODYSIZE_FIELD = 42;

    // **************************Flags**************************/
    /**
     * Flag indicating if dispose() was called before.
     */
    private boolean wasClosed;

    /**
     * Flag indicating if this class was correctly setup.
     */
    private boolean setupSuccess = false;

    // **************************Components*********************/
    /** OpenIGTLink Client socket - socket of the connected Client. */
    private Socket openIGTClient;

    /** Server socket for the communication with state control interface. */
    private ServerSocket openIGTServer;

    /** Output stream for sending data to an IGT client. */
    private OutputStream outStream;

    /** Input stream for receiving data from an IGT server. */
    private InputStream inStream;

    /** The port of the server. */
    private final int serverPort;

    /**
     * Logging mechanism provided by jdk. In case if debug flag is active, all
     * logging output will be directed to a logfile. Otherwise logging output
     * will be displayed on the smartpad.
     */
    private Logger logger = Logger
	    .getLogger(LwrIgtlLogConfigurator.LOGGERS_NAME);

    // *************************Parameters**********************/
    /** The current socket timeout in milliseconds . */
    private int curSocketTimeout;

    // *************************Constructors********************/
    /**
     * Creates and initializes the communication object.
     * 
     * @param port
     *            the port for the communication socket. Supported ports: 49001
     *            - 49005.
     * @param socketTimeout
     *            the initial socket timeout in milliseconds.
     */
    public IGTLComPort(final int port, final int socketTimeout) {
	logger.entering(this.getClass().getName(), "constructor(...)");
	if (port <= MAXIMUM_PORT_NUMBER && port >= MINIMUM_PORT_NUMBER) {
	    serverPort = port;
	} else {
	    throw new IllegalArgumentException("Supported ports are "
		    + MINIMUM_PORT_NUMBER + " to " + MAXIMUM_PORT_NUMBER);
	}
	logger.exiting(this.getClass().getName(), "constructor(...)");

    }

    // ***************************Methods***********************/
    /**
     * Setups all communication related objects. This method blocks until a
     * connection is set up to the igtserver. Input and output streams will be
     * connected immediately and messages can be sent and received.
     * 
     * @throws IOException
     *             when getter of output stream of the client fails.
     */
    public void setup() throws IOException {

	logger.entering(this.getClass().getName(), "setup()");
	try {

	    openIGTServer = ServerSocketFactory.getDefault()
		    .createServerSocket(serverPort);
	    openIGTServer.setReuseAddress(true);
	    logger.finest("Server with port " + serverPort + " installed.");

	    // Set up client connection
	    openIGTClient = openIGTServer.accept();
	    openIGTClient.setSoTimeout(curSocketTimeout);
	    openIGTClient.setTcpNoDelay(true);
	    logger.finest("Client with port " + serverPort + " installed.");

	    // Connect input and output stream, to send and receive messages
	    logger.info(this.getClass().getSimpleName()
		    + " is connecting the in- and outputstreams");
	    outStream = openIGTClient.getOutputStream();
	    inStream = openIGTClient.getInputStream();
	    logger.finest("I/O Streams connected.");

	    // Reset flag, which indicates if this connection has been closed.
	    wasClosed = false;
	    setupSuccess = true;
	    logger.info("IGTL-connection to the server with the ip "
		    + openIGTClient.getInetAddress().toString()
		    + " and the port " + serverPort
		    + " was successfully established.");
	    logger.fine("Setup finished.");
	} catch (Exception e) {
	    logger.log(Level.SEVERE, "Exception caught during setup of "
		    + this.getClass().getSimpleName()
		    + ". Disposing port and throwing new exception.", e);
	    dispose();
	    throw new IllegalStateException(e);
	}
	logger.exiting(this.getClass().getName(), "setup()");

    }

    /**
     * Closes the connection, which was previously established via IGT.
     * 
     * @throws IOException
     *             when closing fails.
     */
    public void dispose() throws IOException {
	logger.entering(this.getClass().getName(), "dispose()");
	if (null != inStream) {
	    try {
		inStream.close();
		logger.finest("Instream closed.");

	    } catch (IOException e) {
		logger.severe("Cannot close instream.");
	    }
	}
	if (null != outStream) {
	    try {
		outStream.close();
		logger.finest("Outstream closed.");

	    } catch (IOException e) {
		logger.severe("Cannot close outstream.");
	    }
	}

	if (null != openIGTServer) {
	    if (!openIGTServer.isClosed()) {
		openIGTServer.close();
		logger.finest("Server closed.");
	    }
	    openIGTServer = null;
	}
	if (null != openIGTClient) {
	    if (!openIGTClient.isClosed()) {
		openIGTClient.close();
		logger.finest("Client closed.");
	    }
	    openIGTClient = null;
	}
	wasClosed = true;
	logger.fine("Disposing finished.");

	logger.exiting(this.getClass().getName(), "dispose()");
    }

    //
    // /**
    // * Sets up the connections again.
    // *
    // * @param newSocketTimeout
    // * a new desired socket timeout in milliseconds.
    // * @throws IOException
    // * when opened connections cannot be closed.
    // */
    // public void restart(final int newSocketTimeout) throws IOException {
    // if (!wasClosed) {
    // dispose();
    // setupSuccess = false;
    // }
    // if (newSocketTimeout < 0) {
    // throw new IllegalArgumentException("The negative timeout "
    // + newSocketTimeout
    // + "ms is not allowed to be set as socket timeout.");
    // }
    //
    // curSocketTimeout = newSocketTimeout;
    // setup();
    // wasClosed = false;
    //
    // }

    /**
     * Sends a message to an IGT client {@linkplain IIGTLMsgInterface}.
     * 
     * @param packet
     *            the packet to be send.
     * @throws IOException
     *             when sending failed.
     */
    public void sendMsg(final IIGTLMsgInterface packet) throws IOException {
	if (packet.getHeader() != null && packet.getBody() != null) {
	    sendBytes(packet.getHeader());
	    sendBytes(packet.getBody());
	} else {
	    throw new NullPointerException(
		    "Paket, which has to be send has either a null header "
			    + "or a null body");
	}

    }

    /**
     * Sends an OpenIGTlink String message. Takes the textual arguments,
     * constructs an {@linkplain IIGTLMsgInterface} and sends it.
     * 
     * @param message
     *            the message text
     * @throws IOException
     *             when sending of the message fails.
     */
    public void sendIGTStringMessage(final String message) throws IOException {
	logger.entering(this.getClass().getName(), "sendIGTStringMessage(...)");

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

	currentMsg = new IGTLMsg();
	currentMsg.init(headerByte, bodyByte);
	logger.fine("IGTL message constructed.");
	sendMsg(currentMsg);
	logger.fine("IGTL message sent.");
	logger.exiting(this.getClass().getName(), "sendIGTStringMessage(...)");
    }

    /**
     * Receives a IGT message.
     * 
     * @return the IGT message or null if no message is available.
     * @throws IOException
     *             reading of input stream fails.
     */
    public synchronized IIGTLMsgInterface receiveMsg() throws IOException {
	logger.entering(this.getClass().getName(), "receiveMsg(...)");
	// Header bytes
	byte[] headerRead = new byte[IGTLheader.IGTL_HEADER_SIZE];
	// Body bytes
	byte[] bodyRead = null;

	if (!setupSuccess) {
	    throw new IllegalStateException(
		    "The connection was not setup properly");
	}

	/*
	 * Read header bytes from stream.
	 */
	if (inStream.available() >= IGTLheader.IGTL_HEADER_SIZE) {
	    inStream.read(headerRead);
	    logger.finest("Message-header received.");
	} else { // no data is available
	    logger.exiting(this.getClass().getName(), "receiveMsg(...)");
	    return null;
	}
	/*
	 * Now read body bytes from stream.
	 */
	int sizeBody = extractBodySize(headerRead);
	bodyRead = new byte[sizeBody];
	if (inStream.available() >= sizeBody) {
	    inStream.read(bodyRead);
	    logger.finest("Message-body received.");

	} else {
	    throw new IllegalStateException("Connection error. Data was lost!");
	}

	/*
	 * Construct the returned object as a IGTL response.
	 */
	IIGTLMsgInterface response = new IGTLMsg();
	response.init(headerRead, bodyRead);
	logger.finest("IGTL-message constructed from received bytes.");
	logger.fine("IGTL-message received of " + headerRead.length
		+ bodyRead.length + " bytes.");
	logger.exiting(this.getClass().getName(), "receiveMsg(...)");
	return response;

    }

    /**
     * Reads the ip address of the client.
     * 
     * @return the ip address.
     */
    public InetAddress getInetAddress() {
	return openIGTClient.getInetAddress();
    }

    /**
     * Reads the port of the client.
     * 
     * @return the port.
     */
    public int getPort() {
	return openIGTClient.getPort();
    }

    /**
     * Extracts the size of the body of an IGT message.
     * 
     * @param header
     *            the whole read header.
     * @return the size of the body in bytes.
     */
    public static int extractBodySize(final byte[] header) {

	// Field in the header which tells how big body is.
	byte[] bodySizeField = new byte[SIZE_OF_BODYSIZE_FIELD];
	/*
	 * extract bodysize field by copying a specific range from the header
	 * int bodySizeField.
	 */
	int m = 0; // iterates over bodySizeField, which gets field
	for (int i = BEGIN_POS_BODYSIZE_FIELD; i < BEGIN_POS_BODYSIZE_FIELD
		+ SIZE_OF_BODYSIZE_FIELD; i++) {
	    bodySizeField[m] = header[i];
	    m++;
	    // use Arrays.copyOfRange...
	    // Arrays.copyOfRange(headerRead, BEGIN_POS_BODYSIZE_FIELD,
	    // BEGIN_POS_BODYSIZE_FIELD + SIZE_OF_BODYSIZE_FIELD - 1);
	}

	/*
	 * Extract the size of the body.
	 */
	BigInteger bodySizeInteger = new BigInteger(bodySizeField);
	return bodySizeInteger.intValue();

    }

    /**
     * In this function the transform message is packed using the SWIGGED
     * igtl_util classes and send to the openIGTClient by calling the
     * sendMessage function.
     * 
     * @param deviceName
     *            - Device Name of the open IGTLink Transform message send to
     *            the visualization software
     * @param transform
     *            - the transformation to be send
     * @throws IOException
     *             when sending fails
     */
    public void sendIGTLTransform(final String deviceName,
	    final float[] transform) throws IOException {
	logger.entering(this.getClass().getName(), "sendIGTLTransform(...)");
	IGTLMsg currentMessage = new IGTLMsg();
	byte[] bodyByte = new byte[IGTLtransform.IGTL_TRANSFORM_SIZE];
	byte[] headerByte = new byte[IGTLheader.IGTL_HEADER_SIZE];
	igtl_header header = new igtl_header();
	header.setVersion(IGTLheader.IGTL_HEADER_VERSION);
	header.setBody_size((BigInteger
		.valueOf(IGTLtransform.IGTL_TRANSFORM_SIZE)));
	header.setName("TRANSFORM");
	header.setDevice_name(deviceName); /* Device name */
	header.setTimestamp(BigInteger.valueOf(System.nanoTime()));
	// BodyBuffer = ByteBuffer.wrap(BodyByte);
	ByteBuffer bodyBuffer = ByteBuffer
		.allocate(IGTLtransform.IGTL_TRANSFORM_SIZE);
	final int size = 12;
	for (int i = 0; i < size; i++) {
	    bodyBuffer.putFloat(transform[i]);
	}

	bodyByte = bodyBuffer.array();
	ByteArr bodyArray = new ByteArr(bodyByte.length);
	for (int i = 0; i < bodyByte.length; i++) {
	    bodyArray.setitem(i, bodyByte[i]);
	}

	ByteArr headerArray = ByteArr.frompointer(IGTLheader.PackHeader(header,
		bodyArray.cast()));
	for (int i = 0; i < IGTLheader.IGTL_HEADER_SIZE; i++) {
	    headerByte[i] = (byte) headerArray.getitem(i);
	}

	currentMessage.init(headerByte, bodyByte);
	sendMsg(currentMessage);
	logger.fine("IGTL-message sent of " + headerByte.length
		+ bodyByte.length + " bytes.");
	logger.exiting(this.getClass().getName(), "sendIGTLTransform(...)");

    }

    /**
     * Sends bytes.
     * 
     * @param bytes
     *            the bytes to be send.
     * @throws IOException
     *             when sending fails.
     */
    private synchronized void sendBytes(final byte[] bytes) throws IOException {
	outStream.write(bytes);
	outStream.flush();
    }

}
