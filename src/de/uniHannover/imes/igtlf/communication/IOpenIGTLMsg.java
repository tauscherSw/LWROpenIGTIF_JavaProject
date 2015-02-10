package de.uniHannover.imes.igtlf.communication;

/**
 * All openIGTL messages contain of a body and a header. This interface gains
 * access to both fields and gives methods for setup of a message.
 *
 */
public interface IOpenIGTLMsg {

    /**
     * Getter for the header of a IGTL message.
     * 
     * @return the header as byte array.
     */
    byte[] getHeader();

    /**
     * Getter for the body of a IGTL message.
     * 
     * @return the body as byte array.
     */
    byte[] getBody();

    /**
     * Initialized a openIGTL message with the parameters.
     * 
     * @param header
     *            the header of the message, which has to be initialized.
     * @param body
     *            the body of the message, which has to be initialized.
     */
    void init(byte[] header, byte[] body);

}
