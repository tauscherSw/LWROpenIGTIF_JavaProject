package de.uniHannover.imes.igtIf.communication.layer;

import de.uniHannover.imes.igtIf.communication.IOpenIGTLMsg;

/**
 * Represents all possible actions a class can have for communication via igtl
 * 
 * @author Tobi
 *
 */
public interface IIGTLCommunicator {

    public static enum Channel {
	CONTROL, VISUAL;

    }

    public void sendMsg(final IOpenIGTLMsg msg, final Channel channel);

    public IOpenIGTLMsg receiveMsg();

}
