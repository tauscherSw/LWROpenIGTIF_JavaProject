/*=========================================================================

  Program:   StateMachineApplication
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

import java.util.concurrent.TimeUnit;

/**
 * This class holds static method(s) for project specific sleeping.
 */
public final class SleepUtil {

    // *************************Constructors********************/
    /**
     * Privatized constructor, because this utility-class shouldn't be
     * instantiated.
     */
    private SleepUtil() {
	super();
    }

    // ***************************Methods***********************/
    /**
     * Sleeps for a specified period of time. It should be called every
     * iteration in the main loop. The time to sleep is calculated according to
     * the loop iteration duration. This method is used for stability
     * enhancement.
     * 
     * @param startTimeNanos
     *            the start time of the loop
     * @param cycleTimeToleranceMs
     *            the tolerance border. If {@code MS_TO_SLEEP} -
     *            {@code cycleTimeToleranceMs} is bigger than the loop-iteration
     *            runtime, then sleeping is necessary.
     * @param cycleTime
     *            the desired cycle time for a loop iteration in milliseconds.
     * @throws InterruptedException
     *             when sleeping of this thread was interrupted.
     */
    public static void cyclicSleep(final long startTimeNanos,
	    final int cycleTimeToleranceMs, final int cycleTime)
	    throws InterruptedException {
	long runtime = (long) ((System.nanoTime() - startTimeNanos));
	long maxAllowedCycleTime = TimeUnit.MILLISECONDS.toNanos(cycleTime
		- cycleTimeToleranceMs);

	if (runtime < maxAllowedCycleTime) {

	    long sleepTimeMs = Math.round(Math
		    .floor((maxAllowedCycleTime - runtime) * 10e-7));
	    int sleepTimeNano;
	    if (sleepTimeMs != 0) {
		sleepTimeNano = (int) (maxAllowedCycleTime - runtime - TimeUnit.MILLISECONDS
			.toNanos(sleepTimeMs));
	    } else {
		sleepTimeNano = 0;
	    }
	    Thread.sleep(sleepTimeMs, sleepTimeNano);

	}
    }

}
