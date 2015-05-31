/*=========================================================================

  Program:   StatisicalTimer
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

package de.uniHannover.imes.igtIf.util;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;

/**
 * Class for recording statistics about loop timing. Use this class by
 * initiating as usual and then call {@linkplain loopBegin()} and {@linkplain
 * loopEnd()} for recording the timing.
 */
public class StatisticalTimer {

    // **************************Components*********************/
    /** Saves the timing of one loop. */
    private OneTimeStep step;
    /** Timer object for evaluating statistics over loop iterations. */
    private StatisticTimer timer;
    /**
     * The time the according thread has to sleep every iteration. This time
     * will be compensated in statistics.
     */
    private int millisectoSleep;

    // *************************Constructors********************/
    /**
     * Creates a statistical timer, which is used to record the timing
     * statistics of a loop.
     * 
     * @param msToSleep
     *            if the loop, which's timing statistics have to be recorded has
     *            a specified sleep time every iteration, this is compensated
     *            for statistics.
     */
    public StatisticalTimer(final int msToSleep) {
	millisectoSleep = msToSleep;
	init();

    }

    // ***************************Methods***********************/
    /**
     * Creates a new timer object. Should only be called within the constructor.
     */
    private void init() {
	timer = new StatisticTimer();
    }

    /**
     * Call this method at every beginning of the loop, which's timing
     * statistics have to be recorded.
     */
    public final void loopBegin() {
	step = timer.newTimeStep();
    }

    /**
     * Call this method at every ending of the loop, which's timing statistics
     * have to be recorded.
     */
    public final void loopEnd() {
	step.end();
    }

    /**
     * Prints statistics for the loop iterations recorded up to the calling of
     * this method.<b>This method is adapted to the openIGTLink project.</b>
     * 
     * @return the statistics as a string if bad communication quality was
     *         detected, otherwise null.
     */
    public final String printOpenIGTLStatistics() {
	if (timer.getMaxTimeMillis() >= 10 * millisectoSleep
		|| timer.getMeanTimeMillis() >= 2 * millisectoSleep) {
	    return "Bad communication quality.";
	} else if ((timer.getMaxTimeMillis() > 3.0 * millisectoSleep && timer
		.getMaxTimeMillis() < 10.0 * millisectoSleep)
		|| (timer.getMeanTimeMillis() > millisectoSleep + 5 && timer
			.getMeanTimeMillis() < 2 * millisectoSleep)) {
	    return "Warning bad communication quality!";
	}
	return null;

    }

    /**
     * Resets the statistical timer and especially the statistics recorded up to
     * the calling of this method.
     */
    public final void reset() {
	init();
    }

    /**
     * Getter for the mean execution time of the loop up to now.
     * 
     * @return the mean time of the loop in millis.
     */
    public final double getMeanTimeMillis() {
	return timer.getMeanTimeMillis();
    }

    /**
     * Returns the statistics recorded by this timing class up to the calling of
     * this method. It gives information about the maximum-, the mean- and the
     * minimum-time.
     * 
     * @return a string holding the information described above.
     */
    public final String getOverallStatistics() {
	return "Statistics for loop iteration time: \n" + "minimum-time [ms]: "
		+ timer.getMinTimeMillis() + "\n" + "mean-time [ms]: "
		+ timer.getMeanTimeMillis() + "\n" + "maximum-time [ms]: "
		+ timer.getMaxTimeMillis();
    }

}
