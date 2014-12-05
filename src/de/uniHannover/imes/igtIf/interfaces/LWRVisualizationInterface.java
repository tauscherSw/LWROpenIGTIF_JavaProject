package de.uniHannover.imes.igtIf.interfaces;

import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.util.concurrent.Semaphore;

import javax.net.ServerSocketFactory;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;


import org.medcare.igtl.messages.OpenIGTMessage;
import org.medcare.igtl.messages.TransformMessage;
import org.medcare.igtl.util.Header;

import Jama.Matrix;

import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;

/**
 * This Class for the Communication with a Visualization system using the opnIGTLink protocol is based on the igtlink4j class developed at the WPI.
 * @author Sebastian Tauscher
 * @see 
 */
public class LWRVisualizationInterface extends Thread {
	OneTimeStep aStep;
	/**
	 * Statistic Timer for the Visualization Interface Thread.
	 */
	public StatisticTimer Visualtiming = new StatisticTimer();
	/**
	 * OpenIGTLink Client socket - socket of the connected Client
	 */
	 private java.net.Socket openIGTClient = null;
	 /**
	  * openIGTLink visualization server socket
	  */
     private ServerSocket openIGTServer;
     /**
      * Outpu stream for sending the currant transformation or joint angles to visualzation software
      */
     private OutputStream outstr;
     /**
      * Flag to indicate if an Error accured during the last cycle
      */
     private boolean ErrorFlag= false;
     
     /**
      * Enum for the client status. Possible states are connected and disconnected
      *
      */
     public static enum ClientStatus {CONNECTED, DISCONNECTED }; //possible client states
     
     /**
      * current client status. Possible states are connected and disconnected
      *
      */
     public ClientStatus currentStatus = ClientStatus.DISCONNECTED; //start as stopped status
     /**
      * Enum for the type of date requested from the visualization Software (Image space, roboter base COF, joint space)
      *       
      */
     public static enum VisualIFDatatypes {IMAGESPACE, ROBOTBASE,JOINTSPACE}; //possible client states
     
     /**
      * Current selected data type to be send to the robot.
      */
     public VisualIFDatatypes datatype = VisualIFDatatypes.ROBOTBASE; //start as stopped status
     
     /**
      * Flag to indicate if the Visualization interface is set active or not.
      */
	public boolean VisualActive = false;
	 /**
     * Flag to indicate if the Visualization interface is running or if the thread is stopped.
     */
	public boolean VisualRun = false;
	/**
	 * Current Cartesian position in robot base coordinate system of the robot
	 */
	public MatrixTransformation cartPose_StateM= null;
	/**
	 * Transformation from robot base coordinate system to image space coordinate system
	 */
	public MatrixTransformation T_IMGBASE_StateM= null;
	/**
	 * Current Joint positions
	 */
	public JointPosition jntPose_StateM = null;
	
	/**
	 * Working copy of the transformation from robot base coordinate system to image space coordinate system.
	 */
	public MatrixTransformation T_IMGBASE= null;
	
	/**
	 * Working copy of the Current Cartesian position in robot base coordinate system of the robot
	 */
	private MatrixTransformation cartPose= null;
	
	/**
	 * Working copy of the Current Joint6 positionof the robot
	 */
	private JointPosition jntPose = null;
	
	/**
	 * Error message string
	 */
	public String ErrorMessage = "";
	
	/**
	 * Semaphore for secure acces to the shared variables
	 */
	public Semaphore VisualSemaphore = new Semaphore(1,true);
	
	/**
     * portnumber for the communication with visualization software e.g. 3D Sliacer. Possible ports 49001 - 49005 
     */
	public int port = 49002;
	
	/**
	 * cycle time of the visualization interface thread. Default value is 25 ms
	 */
	public int millisectoSleep = 25;
	/**
	 * in this String the last printed error message is saved to check if it is error message has already been printed
	 */
	private String LastPrintedError = "";
	
	private int njoint = 0;
	
	private MatrixTransformation [] T_DH = new MatrixTransformation [8];
	
    /**
     * Starts the listening server on the defined port.
     * @param port the port for the communication with state control
     * @throws IOException
     */
	public void ConnectServer(int port) throws IOException{
    	stopServer();
        try {
        	ServerSocketFactory serverSocketFactory = ServerSocketFactory.getDefault();
        	openIGTServer = serverSocketFactory.createServerSocket(this.port);
        	openIGTServer.setReuseAddress(true);
        	System.out.println("Visualization interface server socket succesfully created (port " + this.port + ")");
          
        } catch (IOException e) {
        	System.out.println("Could not Connect to Visualization interface server");
                throw e;
        }
    }
	  
    /**
     * Stops the listening OpenIGTLink server
     * 
     */
	 public void stopServer(){
			if(openIGTServer!= null){
				try {
					openIGTServer.close();
					//openIGTServer= null;
					System.out.println("Visualization interface server stopped");
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
			//socket = null;
			//currentStatus = ServerStatus.STOPPED;
		}
	 public LWRVisualizationInterface(){
		 setDaemon(true);
	 }
	 /**
	  * In this function the Matrixtransformation for each Joint is calculated from the set of denavit hardenberg parameter
	  */
		 public void SetDHTransformation(JointPosition q){
			// double [] d = {0.31 0.4 0.39 0.078};
			
			 MatrixTransformation T_1 = MatrixTransformation.of(Vector.of(0, 0, 0)     , com.kuka.roboticsAPI.geometricModel.math.Matrix.ofRowFirst(Math.cos(q.get(0)), -Math.sin(q.get(0)), 0, Math.sin(q.get(0)), Math.cos(q.get(0)), 0, 0, 0, 1));
			 MatrixTransformation T_2 = MatrixTransformation.of(Vector.of(0, 0, 310)   , com.kuka.roboticsAPI.geometricModel.math.Matrix.ofRowFirst(Math.cos(q.get(1)), -Math.sin(q.get(1)),0, 0,0,-1,  Math.sin(q.get(1)), Math.cos(q.get(1)), 0));
			 MatrixTransformation T_3 = MatrixTransformation.of(Vector.of(0, 200, 0)   , com.kuka.roboticsAPI.geometricModel.math.Matrix.ofRowFirst(Math.cos(q.get(2)), -Math.sin(q.get(2)),0, 0,0, 1, - Math.sin(q.get(2)), -Math.cos(q.get(2)), 0));
			 MatrixTransformation T_4 = MatrixTransformation.of(Vector.of(0, 0, 200)   , com.kuka.roboticsAPI.geometricModel.math.Matrix.ofRowFirst(Math.cos(q.get(3)), -Math.sin(q.get(3)),0, 0,0, 1, - Math.sin(q.get(3)), -Math.cos(q.get(3)), 0));
			 MatrixTransformation T_5 = MatrixTransformation.of(Vector.of(0, -390/2, 0), com.kuka.roboticsAPI.geometricModel.math.Matrix.ofRowFirst(Math.cos(q.get(4)), -Math.sin(q.get(4)),0, 0,0,-1,  Math.sin(q.get(4)), Math.cos(q.get(4)), 0));
			 MatrixTransformation T_6 = MatrixTransformation.of(Vector.of(0, 0, 390/2) , com.kuka.roboticsAPI.geometricModel.math.Matrix.ofRowFirst(Math.cos(q.get(5)), -Math.sin(q.get(5)),0, 0,0,-1,  Math.sin(q.get(5)), Math.cos(q.get(5)), 0));
			 MatrixTransformation T_7 = MatrixTransformation.of(Vector.of(0,    78, 0) , com.kuka.roboticsAPI.geometricModel.math.Matrix.ofRowFirst(Math.cos(q.get(6)), -Math.sin(q.get(6)),0, 0,0, 1, - Math.sin(q.get(6)), -Math.cos(q.get(6)), 0));
			 MatrixTransformation T_8 = MatrixTransformation.of(Vector.of( -5, 30.7, 216) , com.kuka.roboticsAPI.geometricModel.math.Matrix.ofRowFirst(1, 0,0, 0,1, 0,0, 0, 1));
			 T_DH[0]= T_1;
			 T_DH[1]=  T_DH[0].compose(T_2);
			 T_DH[2]=  T_DH[1].compose(T_3);
			 T_DH[3]=  T_DH[2].compose(T_4);
			 T_DH[4]=  T_DH[3].compose(T_5);
			 T_DH[5]=  T_DH[4].compose(T_6);
			 T_DH[6]=  T_DH[5].compose(T_7);
			 T_DH[7]=  T_DH[6].compose(T_8);

			 
			 
		 }
	    /**
	    * Main function of the Visualization Interface. In this function the server is initialized and a packet handler is started.
	    * In a loop with a cycle time of 20 ms the new Command String is received and the Acknowledgment String send.
	    * @param IP the IP address of the State Control Computer, e.g. the system where your Slicer/Matlab GUI is running.
	    * @param port the port number used for the communication with the State Control Computer.
	     * @return 
	    * @see 
	    **/
	    public void run() {
			// TODO Automatisch generierter Methodenstub
	    	

	    	
			
			//Initializing the Communication with the Visualization Software
			try {
				//Set up server
				ConnectServer(port);
				VisualRun = true;
				VisualActive = true;
				openIGTClient = openIGTServer.accept();
				openIGTClient.setTcpNoDelay(true);
				//openIGTClient.setSoTimeout(20);
		        this.outstr =openIGTClient.getOutputStream();
	            this.currentStatus = ClientStatus.CONNECTED; 
	            System.out.println("Visualization interface client connected ( " + openIGTClient.getInetAddress()+ ", " + openIGTClient.getPort() +")");
				
			}catch (Exception e) {
				// TODO Auto-generated catch block
				ErrorFlag = true;
				ErrorMessage = "Couldn't connect to Visualization interface server!";
			}
			while(VisualRun){
			    long startTimeStamp = (long) (System.nanoTime());
			    int startTimeStamp_nanos= (int) (System.nanoTime()-startTimeStamp*1000000);
				aStep = Visualtiming.newTimeStep();
				if(VisualActive){
					//Get new data from State machine
					try {
						VisualSemaphore.acquire();
						jntPose = jntPose_StateM;
						cartPose= cartPose_StateM;
						if(datatype.equals(VisualIFDatatypes.IMAGESPACE)){
							T_IMGBASE =T_IMGBASE_StateM;
							cartPose = T_IMGBASE.compose(cartPose);
						}
						VisualSemaphore.release();
						//Send the transform to Visualization
						
					} catch (InterruptedException e) {
						ErrorFlag = true;
						ErrorMessage = "Unable to Acquire Visual Semaphore";
					}
					
					if( !openIGTClient.isClosed()){
						SendTransform(cartPose, jntPose);
					}
					
					if (ErrorFlag){
						if (!ErrorMessage.equals(LastPrintedError)){
							System.out.println(ErrorMessage);
							LastPrintedError = ErrorMessage;
						}
					}else{
						LastPrintedError = "";
					}
					
				}
				//Set the Module in Sleep mode for stability enhancement
				long curTime = (long) ((System.nanoTime() - startTimeStamp)/1000000.0);
				int curTime_nanos = (int) ((System.nanoTime() - startTimeStamp_nanos) -curTime*1000000.0);
				if(curTime<millisectoSleep){
					//ThreadUtil.milliSleep((long) Math.floor((millisectoSleep-1 -  curTime)));
					try {
						Thread.sleep(millisectoSleep-curTime, curTime_nanos);
					} catch (InterruptedException e) {
						// TODO Automatisch generierter Erfassungsblock
						e.printStackTrace();
					}
				}
				  aStep.end();
				//StepTime = (System.nanoTime() - startTime)/1000000.0;
				/*	try {
						if( StepTime<millisectoSleep -1) {
							Thread.sleep((long) Math.floor((millisectoSleep -1 -  StepTime)));
						}
					} catch (InterruptedException e) {
						ErrorFlag = true;
						ErrorMessage = "Visual Thread Sleep failed!!";
					}*/
			}
		  
		}
		
	 
/**
 * In this function the tranform message is packed and send to the openIGTClient by calling the sendMessage function.
 * @param deviceName - Device Name of the open IGTLink Transform message send to the visualization software
 * @param t - the transformation to be send
 */
		
		
		public void pushTransformMessage( String deviceName, TransformNR t){
			TransformMessage transMsg = new TransformMessage(deviceName, t.getPositionArray(), t.getRotationMatrixArray());
			transMsg.PackBody();
			try {
				sendMessage(transMsg);
			} catch (Exception e) {
				// TODO Automatisch generierter Erfassungsblock
				e.printStackTrace();
			}
		}
		
		public void sendMessage(OpenIGTMessage message) throws Exception {
			// TODO Auto-generated method stub
			sendMessage(message.getHeader(), message.getBody());
			//System.out.println("Message: Header=" + message.getHeader().toString() + " Body=" + message.getBody().toString());
		}
		public void sendMessage(Header header, byte[] body) throws Exception {
			sendBytes(header.getBytes());
			sendBytes(body);
			//System.out.println("Sending Message: Header=" + header.toString() + " Body=" + body.toString());
		}

	    /***************************************************************************
	     * Sends bytes
	     * <p>
	     * 
	     * @throws IOException
	     *             - Exception in I/O.
	     *             <p>
	     * @param bytes
	     *            - byte[] array.
	     **************************************************************************/
	    final public synchronized void sendBytes(byte[] bytes) throws IOException {
	            outstr.write(bytes);
	            outstr.flush();
	    }

	    /***************************************************************************
	     * Interrupt this thread
	     **************************************************************************/
		
	
   
	
	/**
	 * Sending the Cartesian or Joint position of the robot.
	 * @param T_curPose the current position of the robot
	 */
	
	public void SendTransform(MatrixTransformation T_curPose, JointPosition curJntPose){
		double[] t_tmp = new double [3];
		 t_tmp[0]=T_curPose.getTranslation().getX();
		 t_tmp[1]= T_curPose.getTranslation().getY();
		 t_tmp[2]= T_curPose.getTranslation().getZ();
		double[][] R_tmp = null; 
		R_tmp = new double[3][3];
		// Checking what data type was requested
		if(datatype.name().contentEquals((VisualIFDatatypes.JOINTSPACE.name()))){ //if joint space data was requested use the first seven values for the joint angles
			
			SetDHTransformation(jntPose);
			if(njoint >= 8){
				njoint = 0;
			}
			 t_tmp[0]= T_DH[njoint].getTranslation().getX();
			 t_tmp[1]= T_DH[njoint].getTranslation().getY();
			 t_tmp[2]= T_DH[njoint].getTranslation().getZ();
			 for(int i=0; i<3; i++){
					for(int j=0; j<3; j++){
						R_tmp[i][j]=T_DH[njoint].getRotationMatrix().get(i, j);
					}
			}
			njoint++;
			TransformNR T_tmp = new TransformNR(t_tmp,R_tmp);	
			pushTransformMessage("T_" + njoint, T_tmp);
			
			
		}else if (datatype.name().contentEquals((VisualIFDatatypes.IMAGESPACE.name()))){// if imagespace data was requested the current robot position in image space is calculated and send 
			T_curPose = T_IMGBASE.compose(T_curPose);
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++){
					R_tmp[i][j]=T_curPose.getRotationMatrix().get(i, j);
				}
			}
			TransformNR T_tmp = new TransformNR(t_tmp,R_tmp);
			pushTransformMessage("CurCartPose_ROB", T_tmp);
			
		}else{//if the robot base pose was requested the current position in robot space is send
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++){
					R_tmp[i][j]=T_curPose.getRotationMatrix().get(i, j);
				}
			}
			TransformNR T_tmp = new TransformNR(t_tmp,R_tmp);
			pushTransformMessage("CurCartPose_ROB", T_tmp);
		}
	}
	
	
}
