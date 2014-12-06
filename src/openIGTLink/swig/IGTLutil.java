/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.0
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package openIGTLink.swig;

public class IGTLutil implements IGTLutilConstants {
    public static int igtl_is_little_endian() {
	return IGTLutilJNI.igtl_is_little_endian();
    }

    public static java.math.BigInteger crc64(SWIGTYPE_p_unsigned_char data,
	    java.math.BigInteger len, java.math.BigInteger crc) {
	return IGTLutilJNI.crc64(SWIGTYPE_p_unsigned_char.getCPtr(data), len,
		crc);
    }

    public static long igtl_nanosec_to_frac(long nanosec) {
	return IGTLutilJNI.igtl_nanosec_to_frac(nanosec);
    }

    public static long igtl_frac_to_nanosec(long frac) {
	return IGTLutilJNI.igtl_frac_to_nanosec(frac);
    }

    public static void igtl_message_dump_hex(SWIGTYPE_p_FILE stream,
	    SWIGTYPE_p_void message, int max_size) {
	IGTLutilJNI.igtl_message_dump_hex(SWIGTYPE_p_FILE.getCPtr(stream),
		SWIGTYPE_p_void.getCPtr(message), max_size);
    }

    public static long igtl_get_scalar_size(int type) {
	return IGTLutilJNI.igtl_get_scalar_size(type);
    }

}
