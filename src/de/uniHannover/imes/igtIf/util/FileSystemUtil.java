package de.uniHannover.imes.igtIf.util;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Enumeration;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * This class provides static methods for file-operations on the robot
 * controller.
 */
public class FileSystemUtil {

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

	/*
	 * Setup the variables needed for the algorithm
	 */
	JarFile jFile = new JarFile(sourceJar);
	JarEntry jEntry = null;
	boolean fileFoundFlag = false;
	FileSystem fs = FileSystems.getDefault();
	Path outputFile = fs.getPath(destinationFile.getAbsolutePath());

	/*
	 * Begin search for file in jar-archive and copy it.
	 */
	try {
	    Enumeration<JarEntry> jEntries = jFile.entries();

	    // Search for the file
	    while (jEntries.hasMoreElements()) {
		jEntry = jEntries.nextElement();

		// file found
		if (jEntry.getName().equals(filename)) {
		    InputStream dllStream = jFile.getInputStream(jEntry);
		    Files.copy(dllStream, outputFile,
			    StandardCopyOption.REPLACE_EXISTING);
		    System.out.println("Copied " + filename + " from "
			    + sourceJar.getName() + " to "
			    + destinationFile.getAbsolutePath());
		    fileFoundFlag = true;
		    break;

		}

	    }
	    if (!fileFoundFlag) {
		throw new IllegalArgumentException("The file " + filename
			+ " cant be found in " + sourceJar.getName());
	    }

	} catch (IOException e) {
	    System.out.println("Failed to copy " + filename + " from "
		    + sourceJar.getName() + " to "
		    + destinationFile.getAbsolutePath());
	    throw e;
	} finally {
	    jFile.close();
	}

    }

}
