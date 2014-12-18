/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.0
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package openIGTLink.swig;

public class igtl_point_element {
    private long swigCPtr;
    protected boolean swigCMemOwn;

    protected igtl_point_element(long cPtr, boolean cMemoryOwn) {
	swigCMemOwn = cMemoryOwn;
	swigCPtr = cPtr;
    }

    protected static long getCPtr(igtl_point_element obj) {
	return (obj == null) ? 0 : obj.swigCPtr;
    }

    protected void finalize() {
	delete();
    }

    public synchronized void delete() {
	if (swigCPtr != 0) {
	    if (swigCMemOwn) {
		swigCMemOwn = false;
		IGTLpointJNI.delete_igtl_point_element(swigCPtr);
	    }
	    swigCPtr = 0;
	}
    }

    public void setName(String value) {
	IGTLpointJNI.igtl_point_element_name_set(swigCPtr, this, value);
    }

    public String getName() {
	return IGTLpointJNI.igtl_point_element_name_get(swigCPtr, this);
    }

    public void setGroup_name(String value) {
	IGTLpointJNI.igtl_point_element_group_name_set(swigCPtr, this, value);
    }

    public String getGroup_name() {
	return IGTLpointJNI.igtl_point_element_group_name_get(swigCPtr, this);
    }

    public void setRgba(SWIGTYPE_p_unsigned_char value) {
	IGTLpointJNI.igtl_point_element_rgba_set(swigCPtr, this,
		SWIGTYPE_p_unsigned_char.getCPtr(value));
    }

    public SWIGTYPE_p_unsigned_char getRgba() {
	long cPtr = IGTLpointJNI.igtl_point_element_rgba_get(swigCPtr, this);
	return (cPtr == 0) ? null : new SWIGTYPE_p_unsigned_char(cPtr, false);
    }

    public void setPosition(SWIGTYPE_p_float value) {
	IGTLpointJNI.igtl_point_element_position_set(swigCPtr, this,
		SWIGTYPE_p_float.getCPtr(value));
    }

    public SWIGTYPE_p_float getPosition() {
	long cPtr = IGTLpointJNI
		.igtl_point_element_position_get(swigCPtr, this);
	return (cPtr == 0) ? null : new SWIGTYPE_p_float(cPtr, false);
    }

    public void setRadius(float value) {
	IGTLpointJNI.igtl_point_element_radius_set(swigCPtr, this, value);
    }

    public float getRadius() {
	return IGTLpointJNI.igtl_point_element_radius_get(swigCPtr, this);
    }

    public void setOwner(String value) {
	IGTLpointJNI.igtl_point_element_owner_set(swigCPtr, this, value);
    }

    public String getOwner() {
	return IGTLpointJNI.igtl_point_element_owner_get(swigCPtr, this);
    }

    public igtl_point_element() {
	this(IGTLpointJNI.new_igtl_point_element(), true);
    }

}