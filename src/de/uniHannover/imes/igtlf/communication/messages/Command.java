package de.uniHannover.imes.igtlf.communication.messages;

/**
 * This class represents one command received via IGTL.
 */
public class Command {

    /** The unified ID of the corresponding command. */
    private final long uidLocal;

    /** The command string. */
    private final String cmdStringLocal;

    /**
     * Creates a command object with the two corresponding parameters uid and
     * command string.
     * 
     * @param uid
     *            the unified id corresponding to the command string.
     * @param cmd
     *            the command string.
     */
    public Command(final long uid, final String cmd) {
	if (cmd != null) {
	    uidLocal = uid;
	    cmdStringLocal = cmd; // TODO implement all valid cmds and check.
	} else {
	    throw new NullPointerException("Command parameter is null");
	}
    }

    /**
     * Getter for the unified id of this command object.
     * 
     * @return the unified id of this command object.
     */
    public final long getUid() {
	return uidLocal;
    }

    /**
     * Getter of the command string of this command object.
     * 
     * @return the command string.
     */
    public final String getCmdString() {
	return cmdStringLocal;
    }

}