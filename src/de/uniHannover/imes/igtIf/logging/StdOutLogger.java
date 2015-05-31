/*=========================================================================

  Program:   StdOutLogger
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

package de.uniHannover.imes.igtIf.logging;

import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;

/**
 * Logger according to the robotics-API interface {@link ITaskLogger}. This
 * logger just prints all messages and exceptions to System.out or System.err
 * output.
 * 
 */
public class StdOutLogger implements ITaskLogger {
    
    
    //***************************Methods***********************/
    @Override
    public final void error(final String msg) {
	System.err.println(msg);

    }

    @Override
    public final void error(final String msg, final Throwable thrown) {
	System.err.println(msg + " " + thrown.getMessage());

    }

    @Override
    public final void fine(final String msg) {
	System.out.println(msg);

    }

    @Override
    public final void info(final String msg) {
	System.out.println(msg);

    }

    @Override
    public final void warn(final String msg) {
	System.out.println(msg);

    }

    @Override
    public final void warn(final String msg, final Throwable thrown) {
	System.out.println(msg + " " + thrown.getMessage());

    }

}
