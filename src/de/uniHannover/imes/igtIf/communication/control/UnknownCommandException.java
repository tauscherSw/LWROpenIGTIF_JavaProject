package de.uniHannover.imes.igtIf.communication.control;

/**
 * This class represents all runtime exception, which occur during processing
 * commands received via IGTL.
 */
public class UnknownCommandException extends RuntimeException {

    /**
     * Creates an exception which indicates a failure during processing of an
     * IGTL command message.
     * 
     * @param string
     *            the exception message.
     */
    public UnknownCommandException(final String string) {
	super(string);
    }

    /**
     * Generated serial version unified id.
     */
    private static final long serialVersionUID = 2224722643047058519L;

}
