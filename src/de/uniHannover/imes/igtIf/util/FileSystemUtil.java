/*=========================================================================

  Program:   FileSystemUtil
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

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * This class provides static methods for file-operations on the robot
 * controller.
 */
public final class FileSystemUtil {

    // *************************Constructors********************/
    /** Privatized constructor, because this class shouldn't be instantiated. */
    private FileSystemUtil() {
	super();
    }

    // ***************************Methods***********************/
    /**
     * Extracts a file from a jar archive. This means this methods searches for
     * this file and copies it to a specified destination
     * 
     * @param sourceJar
     *            the jar-file, which holds the file to be searched for.
     * @param destinationFile
     *            the desired absolute path (and full name) of the local copy *
     * @param filename
     *            the name, the algorithm is looking for in the jar-archive
     * @throws IOException
     *             if extraction of the file fails.
     */
    public static void extractFileFromJar(final File sourceJar,
	    final File destinationFile, final String filename)
	    throws IOException {

	/*
	 * Preliminary checks if arguments are correct
	 */
	if (!sourceJar.exists() || !sourceJar.isFile()) {
	    throw new IllegalArgumentException(sourceJar.getAbsolutePath()
		    + " does not exist or is no file.");
	}
	if (destinationFile.isDirectory()) {
	    throw new IllegalArgumentException("the argument "
		    + destinationFile.getAbsolutePath() + " is no file");
	}
	if (!destinationFile.exists()) {
	    if (!destinationFile.createNewFile()) {
		throw new IOException("Failed to create "
			+ destinationFile.getAbsolutePath());
	    }

	}

	/*
	 * Setup the variables needed for the algorithm
	 */
	JarFile jFile = new JarFile(sourceJar);
	JarEntry jEntry = null;
	FileOutputStream outPut = null;
	InputStream dllStream = null;

	try {

	    /*
	     * Begin search for file in jar-archive and copy it.
	     */

	    jEntry = jFile.getJarEntry(filename);
	    if (jEntry == null) {
		jFile.close();
		throw new IllegalArgumentException("The file " + filename
			+ " cant be found in " + sourceJar.getName());
	    }

	    dllStream = jFile.getInputStream(jEntry);
	    outPut = new FileOutputStream(destinationFile);

	    // Check write access, check if file is lockable
	    if (!destinationFile.canWrite() || !destinationFile.setReadOnly()) {
		jFile.close();
		outPut.close();
		throw new IOException("Cannot write & lock file "
			+ destinationFile.getAbsolutePath());
	    }

	    while (true) {
		int bytesRead = dllStream.read();
		// check if all bytes were read.
		if (bytesRead == -1) {
		    break;
		}
		outPut.write(bytesRead);
	    }
	} finally {
	    if (null != outPut) {
		outPut.close();
	    }
	    if (null != dllStream) {
		dllStream.close();
	    }
	    if (null != jFile) {
		jFile.close();
	    }
	    destinationFile.setWritable(true);
	}

    }
}
