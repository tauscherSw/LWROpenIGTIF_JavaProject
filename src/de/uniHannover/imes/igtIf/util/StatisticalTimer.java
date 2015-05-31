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
