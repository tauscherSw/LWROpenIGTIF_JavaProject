/*=========================================================================

  Program:   IGTMessageHandler
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher. Institute of Mechatronics System, Leibniz Universitaet Hannover. All rights reserved.

	See License.txt for more information
	
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
	CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
	EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
	PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
	NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	=========================================================================*/
package de.uniHannover.imes.igtIf.communicationIf;

import java.util.concurrent.Semaphore;

import de.uniHannover.imes.igtIf.application.StateMachineApplication;


/**
 * This Class is handling the display output on the KUKA SmartPad.
 * 
 * @author Sebastian Tauscher
 * @see LWRVisualizationInterface
 * @see LWRStateMachineInterface
 * 
 */
public class IGTMessageHandler extends Thread {

    /**
     * Error Message String for error in the State machine interface.
     */
    public String errorMessage = ""; //TODO design failure other threads access this field.
    /**
     * Name of the thread using this Message handler.
     */
    public String sendername = ""; //TODO design failure other threads access this field.
    /**
     * in this String the last printed error message is saved to check if it is
     * error message has already been printed.
     */
    private String lastPrintedError = "";

    /**
     * Semaphore for save reading and writing the variables.
     */
    public Semaphore messageSemaphore = new Semaphore(1, true); //TODO design failure other threads access this field.
    /**
     * cycle time of the state control interface thread in milliseconds. Default
     * value is 20 ms.
     */
    private static final int MS_TO_SLEEP = 1000; 
    /**
     * Flag indicating if the message handler thread is active.
     */
    private boolean threadAlive = true;

    /**
     * Flag to indicate if this message handler is displaying the errors at the
     * smartPad or not.
     */
    public boolean debugInfos = false; //TODO design failure other threads access this field.

    /**
     * Constructor, which initializes this thread as a daemon.
     */
    public IGTMessageHandler() {
	setDaemon(true);
    };

    /**
     * Finalizes this thread.
     */
    public void finalize() {

    }

    /**
     * Initialize function of the State control Interface. In this function the
     * server is initialized and a packet handler is started. In a loop with a
     * cycle time of 20 ms the new Command String is received and the
     * Acknowledgment String send.
     **/
    public final void run() {

	while (threadAlive) {

	    long startTimeStamp = (long) (System.nanoTime());
	    try {
		messageSemaphore.acquire();
		if (!this.errorMessage.equals(this.lastPrintedError)) {
		    if (debugInfos) {
			System.out.println("From: " + sendername + ": "
				+ this.errorMessage);
		    }
		    this.lastPrintedError = this.errorMessage;
		}
	    } catch (InterruptedException e) {
		// TODO exception concept.
	    }
	    messageSemaphore.release();
	    
	    // Set the Module in Sleep mode for stability enhancement
		try {

		    StateMachineApplication.cyclicSleep(startTimeStamp, 1, MS_TO_SLEEP);
		} catch (InterruptedException e) {
		    // TODO exception concept.
		}
	    }
	}
    }
    
    


