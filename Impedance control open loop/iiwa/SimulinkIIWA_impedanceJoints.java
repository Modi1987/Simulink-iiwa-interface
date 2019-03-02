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
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

 
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;

import lbrExampleApplications.SimulinkIIWADirectServoJoints.UDPServer;


/*
 * SimulinkIIWA interface, soft real-time control with impedence
 * Copyright: Mohammad SAFEEA, 22nd/May/2018
 * License: MIT license
 * 
 * 
 * This is a UDP server that receives motion commands from Simulink.
 * This server works together with the Simulink example: (impedanceJoints.slx)
 * 
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
 * 3- Start the Simulink example (impedanceJoints.slx)
 *    from the external PC.
 * 4- To change the timeout value, change the argument for the instruction
 *    soc.setSoTimeout(10000),  the argument is in milliseconds.
 * 
 */

public class SimulinkIIWA_impedanceJoints extends RoboticsAPIApplication
{
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
        SimulinkIIWA_impedanceJoints app = new SimulinkIIWA_impedanceJoints();
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
    				soc.setSoTimeout(10000); // 5 seconds, if a time out occurred, turn off program
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
    	
    	
    	public double bytesToDouble(byte[] b)
    	{
    		return ByteBuffer.wrap(b).getDouble();
    	}

}
