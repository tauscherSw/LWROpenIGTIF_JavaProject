package de.uniHannover.imes.igtIf.communication;



/**
 * This class represents a message in the IGTL protocol.
 */
public class IGTLMsg implements IOpenIGTLMsg {

    //**************************Components*********************/
    /**
     * The header of the message.
     */
    private byte[] headerData;

    /**
     * The body of the message.
     */
    private byte[] bodyData;
    

    //***************************Methods***********************/
    @Override
    public final byte[] getHeader() {
	return headerData;
    }

    @Override
    public final byte[] getBody() {
	return bodyData;
    }

    @Override
    public final void init(final byte[] header, final byte[] body) {
	headerData = header;
	bodyData = body;
    }

}
