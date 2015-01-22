package de.uniHannover.imes.igtlf.communication;

public interface IGTLMsgInterface {

    byte[] getHeader();

    byte[] getBody();

    void init(byte[] header, byte[] body);

}
