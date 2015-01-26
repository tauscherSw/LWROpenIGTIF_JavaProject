package de.uniHannover.imes.igtlf.communication;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.math.BigInteger;
import java.net.ServerSocket;
import java.net.Socket;
import javax.net.ServerSocketFactory;

import openIGTLink.swig.IGTLheader;

/**
 * This class provides basic methods for sending and receiving messages to and
 * from via IGTLink.
 */
public final class IGTLCommunicator {

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

    /** OpenIGTLink Client socket - socket of the connected Client. */
    private Socket openIGTClient;

    /** Server socket for the communication with state control interface. */
    private ServerSocket openIGTServer;

    /**
     * Output stream for sending data to an IGT client.
     */
    private OutputStream outStream;

    /**
     * Input stream for receiving data from an IGT server.
     */
    private InputStream inStream;

    /** The port of the server. */
    private final int serverPort;

    /**
     * Flag indicating if dispose() was called before.
     */
    private boolean wasClosed;

    /**
     * Creates and initializes the communication object.
     * 
     * @param port
     *            the port for the communication socket. Supported ports: 49001
     *            - 49005.
     * @throws IOException
     *             when set up of the sockets fails.
     */
    private IGTLCommunicator(final int port) throws IOException {
	if (port <= MAXIMUM_PORT_NUMBER && port >= MINIMUM_PORT_NUMBER) {
	    serverPort = port;
	} else {
	    throw new IllegalArgumentException("Supported ports are "
		    + MINIMUM_PORT_NUMBER + " to " + MAXIMUM_PORT_NUMBER);
	}
	setUpCommunication();

    }

    /**
     * Sets up all communication related objects. This method blocks until a
     * connection is set up to the igtserver
     * 
     * @throws IOException
     *             when getter of output stream of the client fails.
     */
    private void setUpCommunication() throws IOException {
	openIGTServer = ServerSocketFactory.getDefault().createServerSocket(
		serverPort);
	openIGTServer.setReuseAddress(true);
	openIGTClient = openIGTServer.accept();
	wasClosed = false;
	outStream = openIGTClient.getOutputStream();
	inStream = openIGTClient.getInputStream();

    }

    /**
     * Getter for the connection state.
     * 
     * @return true if connected, otherwise false.
     */
    public boolean isConncted() {
	if (openIGTServer != null) {
	    return openIGTServer.isClosed();
	} else {
	    return false;
	}
    }

    /**
     * Closes the connection, which was previously established via IGT.
     * 
     * @throws IOException
     *             when closing fails.
     */
    public void dispose() throws IOException {
	openIGTServer.close();
	openIGTServer = null;
	openIGTClient.close();
	openIGTClient = null;
	wasClosed = true;
    }

    /**
     * Sets up the connections again.
     * 
     * @param newSocketTimeout
     *            a new desired socket timeout in milliseconds.
     * @throws IOException
     *             when opened connections cannot be closed.
     */
    public void restart(final int newSocketTimeout) throws IOException {
	if (!wasClosed) {
	    dispose();
	}
	setUpCommunication();
	openIGTClient = openIGTServer.accept();
	openIGTClient.setTcpNoDelay(true);
	openIGTClient.setSoTimeout(newSocketTimeout);

	wasClosed = false;

    }

    /**
     * Sends a message to an IGT client {@linkplain IGTLMsgInterface}.
     * 
     * @param packet
     *            the packet to be send.
     * @throws IOException
     *             when sending failed.
     */
    public void sendSlicerMsg(final IGTLMsgInterface packet) 
	    throws IOException {
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
     * Receives a IGT message.
     * 
     * @return the IGT message.
     * @throws IOException
     *             reading of input stream fails.
     */
    public IGTLMsgInterface receiveMsg() throws IOException {
	// Header bytes
	byte[] headerRead = new byte[IGTLheader.IGTL_HEADER_SIZE];
	// Body bytes
	byte[] bodyRead = null;

	/*
	 * Read header bytes from stream.
	 */
	inStream.read(headerRead);

	/*
	 * Now read body bytes from stream.
	 */
	int sizeBody = extractBodySize(headerRead);
	bodyRead = new byte[sizeBody];
	inStream.read(bodyRead);

	/*
	 * Construct the returned object as a IGTL response.
	 */
	IGTLMsgInterface response = new IGTLMessage();
	response.init(headerRead, bodyRead);
	return response;

    }

    /**
     * Extracts the size of the body of an IGT message.
     * 
     * @param header
     *            the whole read header.
     * @return the size of the body in bytes.
     */
    private int extractBodySize(final byte[] header) {
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
