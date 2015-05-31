/*=========================================================================

  Program:   VisualSenderConfig
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

package de.uniHannover.imes.igtIf.communication.visualization;

import de.uniHannover.imes.igtIf.communication.visualization.VisualizationThread.VisualIFDatatypes;

/**
 * This class represents the configuration for the visualization interface,
 * which sends cyclic data to slicer. The data send depends on this
 * configuration class.
 *
 */
final class VisualSenderConfig {

    //**************************Components*********************/
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

    

    //***************************Methods***********************/
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
