package de.uniHannover.imes.igtIf.communication.visualization;

import de.uniHannover.imes.igtIf.communication.visualization.VisualizationThread.VisualIFDatatypes;

/**
 * This class represents the configuration for the visualization interface,
 * which sends cyclic data to slicer. The data send depends on this
 * configuration class.
 *
 */
final class VisualSenderConfig {

    /**
     * The datatype the position data is requested in. Possible types are listed
     * in the enumeration {@link VisualIFDatatypes}.
     */
    private VisualIFDatatypes requestedDatatype;
    /**
     * The flag indicating if the external tcp force data will be transmitted to
     * slicer.
     */
    private boolean sendTcpForce;

    /**
     * Sets the data for this class at once.
     * 
     * @param typeOfData
     *            the datatype the position data is requested in. Possible types
     *            are listed in the enumeration {@link VisualIFDatatypes}.
     * @param tcpForceFlag
     *            the flag indicating if the external tcp force data will be
     *            transmitted to slicer.
     */
    void setData(final VisualIFDatatypes typeOfData, 
	    final boolean tcpForceFlag) {
	this.requestedDatatype = typeOfData;
	this.sendTcpForce = tcpForceFlag;
    }

    /**
     * Getter for the requested datatype of the position data.
     * 
     * @return the datatype.
     */
    VisualIFDatatypes getRequestedDatatype() {
	return requestedDatatype;
    }

    /**
     * Getter for the flag indicating if the external tcp force data will be
     * transmitted to slicer.
     * 
     * @return the boolean value of the flag.
     */
    boolean getSendTcpForceFlag() {
	return sendTcpForce;
    }

}
