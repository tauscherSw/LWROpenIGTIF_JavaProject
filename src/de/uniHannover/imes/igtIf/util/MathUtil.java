package de.uniHannover.imes.igtIf.util;

/**
 * This class provides mathematical util functions, which cannot be provided by
 * the jdk.
 */
public final class MathUtil {


    // *************************Constructors********************/
    /**
     * Privatized constructor, because this utility-class shouldn't be
     * instantiated.
     */
    private MathUtil() {
	super();
    }
    //***************************Methods***********************/

    /**
     * This method casts a long value to an integer value. An exception will be
     * thrown if the value changes by casting.
     * 
     * @param longValue
     *            the value to be casted to an integer.
     * @return the integer value, which correspond to the long value.
     */
    public static int longToInt(final long longValue) {
	if (longValue < Integer.MIN_VALUE 
		|| 
		longValue > Integer.MAX_VALUE) {
	    throw new IllegalArgumentException(
		    "The value "
			    + longValue
			    + " cannot be casted to an integer without changing its value");
	} else {
	    return (int) longValue;
	}
    }

}
