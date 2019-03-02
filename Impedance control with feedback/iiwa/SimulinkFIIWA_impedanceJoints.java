package lbrExampleApplications;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;
import java.util.StringTokenizer;
import java.util.logging.Logger;

import com.kuka.common.StatisticTimer;
import com.kuka.common.ThreadUtil;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;

 
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;

import lbrExampleApplications.SimulinkFIIWADirectServoCartesian.UDPPublisher;
import lbrExampleApplications.SimulinkIIWADirectServoJoints.UDPServer;

/*
 * SimulinkIIWA interface, soft real-time control with impedence
 * Copyright: Mohammad SAFEEA, 22nd/May/2018
 * License: MIT license
 * 
 * 
 * This is a UDP server that receives motion commands from Simulink.
 * This server works together with the Simulink example: (feedbackImpedanceJoints.slx)
 * 
 * Also this script returns back a feedback about the  (1)actual joints positions,
 * (2)the external torques and (3)the measured torques of all the joints to Simulink.
 * (4) EEF force moment.
 * 
 * Below you can find quick start instructions, for more info, please refer to the youtube
 * video tutorials, a link for which is available in the repository main page.
 * 
 * Hardware setup:
 * 1- KUKA iiwa 7R800 or 14R820
 * 2- External PC
 * 3- A network between the PC and the robot, the connector X66
 *    of the robot shall be used
 * 
 * Required software:
 * 1- MATLAB with Simulink, the code was written using MATLAB2018a, it 
 *    is recommended to use the same version.
 * 2- Sunrise.Workbench, this program will be used to synchronize the java
 *    code to the robot.
 * 3- The Sunrise code was tested on (KUKA Sunrise.OS 1.11.0.7)
 * 
 * Setup instructions:
 * 1- Add this class and the class (SmartServoWithImpedenceSimulinkIIWAinterface.java) 
 *    to new project  using kuka's Sunrsie.Workbench.
 * 2- Add reference to the SmartServo from inside the (StationSetup.cat) file. 
 * 3- Change the following variables according to your tool and stiffness preference:
 * 	* externa_PC_IP: change this variable according to the IP of the PC used for control.
 *  * TRANSLATION_OF_TOOL: translation of tool center point with respect to flange frame.
 *  * massOfTool: mass of the tool
 *  * toolsCOMCoordiantes: coordinates of center of mass of tool with regards to the frame of the flange.
 *  * cStiffness: Cartesian stiffness
 *  * rStiffness: rotational stiffness
 *  * nStiffness: null space stiffness
 * 
 * 
 * 
 * Utilization instructions:
 * 1- Synchronize the project to the controller of the robot.
 * 2- Run this application from the teach-pad of the robot,
 *    This application has a time out of 10 seconds, if a connection
 *    is not established during the 10 seconds interval the server will
 *    turn off.
 * 3- Start the Simulink example (feedbackImpedanceJoints.slx)
 *    from the external PC.
 * 4- To change the timeout value, change the argument for the instruction
 *    soc.setSoTimeout(10000),  the argument is in milliseconds.
 * 
 */

public class SimulinkFIIWA_impedanceJoints extends RoboticsAPIApplication
{
	
	private static String externa_PC_IP="172.31.69.55";
    private LBR _lbr;
    private Controller kuka_Sunrise_Cabinet_1;
    private int _count = 0;

    private int _steps = 0;

    private UDPServer udpServer;
    
    // Tool data
    private static double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static double massOfTool = 1.1;
    private static double[] toolsCOMCoordiantes = { 0, 0, 45 };
    
    // stiffness data
    private static double cStiffness=450; // cartesian stiffness
    private static double rStiffness=40; // rotational stiffness
    private static double nStiffness=20; // null space stiffness

    
    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);
        kuka_Sunrise_Cabinet_1=getController("KUKA_Sunrise_Cabinet_1");
        SmartServoWithImpedenceSimulinkIIWAinterface.smartServoMotionFlag=true;
    }
 /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    private void moveToInitialPosition()
    {
    	_lbr.move(
        		ptp(0., Math.PI / 180 * 30.,0.,-Math.PI / 180 * 80.,0.,Math.PI / 180 * 70., 0.).setJointVelocityRel(0.15));        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        try
        {
        	
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
    }

    /**
     * Main Application Routine
     */
    @Override
    public void run()
    {
        moveToInitialPosition();
        System.out.print("You have ten seconds to establish a connection with iiwa");
        int portTemp=30002;
        udpServer=new UDPServer(portTemp);
        new UDPPublisher(externa_PC_IP);
        try {
			SmartServoWithImpedenceSimulinkIIWAinterface.startRealTimeWithImpedence(_lbr, kuka_Sunrise_Cabinet_1, massOfTool,
					toolsCOMCoordiantes, cStiffness,
					rStiffness, nStiffness);
		} catch (Exception e) {
			// TODO Bloco catch gerado automaticamente
			e.printStackTrace();
		}
    }
   
 
    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        SimulinkFIIWA_impedanceJoints app = new SimulinkFIIWA_impedanceJoints();
        app.runApplication();
    }
 
    public class UDPServer implements Runnable{
    	
    	
    	double stime=0;
		double endtime=0;
		
    	int _port;
    	int vectorSize=7;
    	int packetCounter=0;
    	
    	byte[] buffer=new byte[8*vectorSize];
    	UDPServer(int port)
    	{
    		Thread t=new Thread(this);
    		t.setDaemon(true);
    		t.start();
    		_port=port;
    	}
    	
    	public void run()
    	{
    			System.out.println("Program will terminate if comuncation is not established in 10 seconds");
    			DatagramSocket soc=null;
    			try {
    				soc = new DatagramSocket(_port);
    				soc.setSoTimeout(10000); // 10 seconds, if a time out occurred, turn off program
    				DatagramPacket response=new DatagramPacket(buffer, buffer.length);
    				while(true)
    				{
    					soc.receive(response);
    					packetCounter=packetCounter+1;
    					// String s= new String(buffer,0,response.getLength())
    					 // System.out.println(response.getLength());
    					if(response.getLength()==8*vectorSize)
    					{
    						if(packetCounter==1)
    						{
    							stime=System.currentTimeMillis();
    						}
    						else
    						{
    							endtime=System.currentTimeMillis();
    						}
    						byte[] daB=new byte[8];
    						int counter=0;
    						int jointNum=0;
    						while(counter <8*vectorSize)
    						{
	    						for(int i=0;i<8;i++)
	    						{
	    							daB[7-i]=buffer[counter];
	    							counter=counter+1;
	    						}
	    						SmartServoWithImpedenceSimulinkIIWAinterface.smartServoJpos[jointNum]=bytesToDouble(daB);
	    						jointNum=jointNum+1;	    						
	    						//System.out.println(bytesToDouble(daB));
    						}
    					}
    				}
    			} catch (SocketException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    				System.out.println(e.toString());
    			} catch (IOException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    				System.out.println(e.toString());
    			}
    			if(soc==null)
    			{}
    			else
    			{
    				if(soc.isClosed())
    				{}
    				else
    				{
    					soc.close();
    				}
    			}
    			SmartServoWithImpedenceSimulinkIIWAinterface.smartServoMotionFlag=false; //Turn off main loop 	
   				double time=(endtime-stime)/1000;
    			double rate=packetCounter/time;
    			System.out.println("update rate, packets/second");
    			System.out.println(rate);
    	}
    }
    	

    
    public class UDPPublisher implements Runnable{
    	
    	String ip;
    	UDPPublisher(String ip)
    	{
    		this.ip=ip;
    		Thread t=new Thread(this);
    		t.setDaemon(true);
    		t.start();
    	}
    	public void run()
    	{
    		broadcastData();
    	}
    	private void broadcastData() {
    		// TODO Auto-generated method stub
    		String message="Publishing robot state using UDP at port 30008 to destination: "+ ip;
    		System.out.println(message);
    		message="Streamed message: 7 actual joint angles, 7 measured torques, 7 external torques,";
    		System.out.println(message);
    		message="State streaming frequency is approximatly at 25HZ";
    		System.out.println(message);
    		int port=30008;
    		InetAddress host;
    		int numOfDoubles=7+7+7+6;// 7 actual joint angles, 7 measured torques, 7 external torques, 7 force at flange
    		double[] doubleVals=new double[numOfDoubles];
    		
    		byte[] buffer= new byte[8*numOfDoubles];
    		
    		try {
    			host = InetAddress.getByName(ip);
    			DatagramSocket soc=new DatagramSocket();
    			while(SmartServoWithImpedenceSimulinkIIWAinterface.smartServoMotionFlag)
    			{
    				// Feedback of actual joints positions
    				JointPosition jpos=_lbr.getCurrentJointPosition();
    				int valsCounter=0;
    				for(int i=0;i<7;i++)
    				{
    					doubleVals[valsCounter]=jpos.get(i);
    					valsCounter=valsCounter+1;
    				}
    				// Feedback of measured torques
    				double[] m_tor=_lbr.getMeasuredTorque().getTorqueValues();
    				for(int i=0;i<7;i++)
    				{
    					doubleVals[valsCounter]=m_tor[i];
    					valsCounter=valsCounter+1;
    				}
    				// Feedback of external torques
    				double[] ex_tor=_lbr.getExternalTorque().getTorqueValues();
    				for(int i=0;i<7;i++)
    				{
    					doubleVals[valsCounter]=ex_tor[i];
    					valsCounter=valsCounter+1;
    				}
    				// Feedback of forces at flange    				
    				ForceSensorData f_at_flange=_lbr.getExternalForceTorque(_lbr.getFlange(),World.Current.getRootFrame());				
    				doubleVals[valsCounter]=f_at_flange.getForce().getX();
    				valsCounter=valsCounter+1;
    				doubleVals[valsCounter]=f_at_flange.getForce().getY();
    				valsCounter=valsCounter+1;
    				doubleVals[valsCounter]=f_at_flange.getForce().getZ();
    				valsCounter=valsCounter+1;
    				doubleVals[valsCounter]=f_at_flange.getTorque().getX();
    				valsCounter=valsCounter+1;
    				doubleVals[valsCounter]=f_at_flange.getTorque().getY();
    				valsCounter=valsCounter+1;
    				doubleVals[valsCounter]=f_at_flange.getTorque().getZ();
    				valsCounter=valsCounter+1;
    				
    				int count=0;
    				for(int i=0;i<numOfDoubles;i++)
    				{
    					byte[] bytes = new byte[8];
    					double value=doubleVals[i];
    				    ByteBuffer.wrap(bytes).putDouble(value);
    				    for(int j=0;j<8;j++)
    				    {
    				    	buffer[count]=bytes[7-j];
    				    	count=count+1;
    				    }
    				}
    				
    				DatagramPacket packet= new DatagramPacket(buffer, buffer.length,host, port);
    				soc.send(packet);
    				Thread.sleep(36);
    			}
    			message="Robot state publisher is terminated";
        		System.out.println(message);
    				
    		} catch (UnknownHostException e1) {
    			// TODO Auto-generated catch block
    			//e1.printStackTrace();
    			System.out.println("Error in UDP publisher, 1");
    			System.out.println(e1.toString());
    		} catch (SocketException e) {
    			// TODO Auto-generated catch block
    			//e.printStackTrace();
    			System.out.println("Error in UDP publisher, 2");
    			System.out.println(e.toString());
    		} catch (IOException e) {
    			// TODO Auto-generated catch block
    			//e.printStackTrace();
    			System.out.println("Error in UDP publisher, 3");
    			System.out.println(e.toString());
    		} catch (InterruptedException e) {
    			// TODO Auto-generated catch block
    			//e.printStackTrace();
    			System.out.println("Error in UDP publisher, 4");
    			System.out.println(e.toString());
    		}
    	}

    }
    	
 
    
    	public double bytesToDouble(byte[] b)
    	{
    		return ByteBuffer.wrap(b).getDouble();
    	}

}
