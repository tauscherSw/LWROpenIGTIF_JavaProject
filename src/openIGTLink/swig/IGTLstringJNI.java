/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.0
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package openIGTLink.swig;

public class IGTLstringJNI {
    public final static native int IGTL_SIZEOF_CHAR_get();

    public final static native int IGTL_SIZEOF_INT_get();

    public final static native int IGTL_SIZEOF_SHORT_get();

    public final static native int IGTL_SIZEOF_LONG_get();

    public final static native int IGTL_SIZEOF_FLOAT_get();

    public final static native int IGTL_SIZEOF_DOUBLE_get();

    public final static native int IGTL_TYPE_USE_LONG_LONG_get();

    public final static native int IGTL_SIZEOF_LONG_LONG_get();

    public final static native long new_ByteArr(int jarg1);

    public final static native void delete_ByteArr(long jarg1);

    public final static native short ByteArr_getitem(long jarg1,
	    ByteArr jarg1_, int jarg2);

    public final static native void ByteArr_setitem(long jarg1, ByteArr jarg1_,
	    int jarg2, short jarg3);

    public final static native long ByteArr_cast(long jarg1, ByteArr jarg1_);

    public final static native long ByteArr_frompointer(long jarg1);

    public final static native int IGTL_STRING_HEADER_SIZE_get();

    public final static native void igtl_string_header_encoding_set(long jarg1,
	    igtl_string_header jarg1_, int jarg2);

    public final static native int igtl_string_header_encoding_get(long jarg1,
	    igtl_string_header jarg1_);

    public final static native void igtl_string_header_length_set(long jarg1,
	    igtl_string_header jarg1_, int jarg2);

    public final static native int igtl_string_header_length_get(long jarg1,
	    igtl_string_header jarg1_);

    public final static native long new_igtl_string_header();

    public final static native void delete_igtl_string_header(long jarg1);

    public final static native long igtl_string_get_string_length(long jarg1,
	    igtl_string_header jarg1_);

    public final static native void igtl_string_convert_byte_order(long jarg1,
	    igtl_string_header jarg1_);

    public final static native java.math.BigInteger igtl_string_get_crc(
	    long jarg1, igtl_string_header jarg1_, long jarg2);
}