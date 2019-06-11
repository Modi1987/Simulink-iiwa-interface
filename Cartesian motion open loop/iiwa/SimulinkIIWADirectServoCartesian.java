package lbrExampleApplications;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;
import java.util.StringTokenizer;
import java.util.logging.Logger;

import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;

 
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;


/*
 * SimulinkIIWA interface, soft real-time control in Cartesian position mode
 * Copyright: Mohammad SAFEEA, 22nd/May/2018
 * License: MIT license
 * 
 * This is a UDP server that receives motion commands from Simulink.
 * This server works together with the Simulink example: (smartDirectServoCartesian.slx)
 * 
 * 
 * Below you can find quick start instructions, for more info, please refer to the youtube
 * video tutorials, a link for which is available in the repository main page.
 * 
 * Hardware setup:
 * 1- KUKA iiwa 7R800 only, modifications in Simulink schematic are required for 14R820
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
 * 1- Add this class to a new project  using kuka's Sunrsie.Workbench.
 * 2- Add reference to the Direct/SmartServo from inside the (StationSetup.cat) file. 
 * 3- Change the following variables according to preference:
 * 	* externa_PC_IP: change this variable according to the IP of the PC used for control.
 * 
 * 
 * Utilization instructions:
 * 1- Synchronize the project to the controller of the robot.
 * 2- Run this application from the teach-pad of the robot,
 *    This application has a time out of 10 seconds, if a connection
 *    is not established during the 10 seconds interval the server will
 *    turn off.
 * 3- Start the Simulink example (smartDirectServoCartesian.slx)
 *    from the external PC.
 * 4- To change the timeout value, change the argument for the instruction
 *    soc.setSoTimeout(10000),  the argument is in milliseconds.
 * 
 * 
 * Updated on 11th/June/2019
 * Bug fix (the bug happened only in 14R820)
 * To fix the bug hard-coding of the Cartesian home position was performed.
 */


public class SimulinkIIWADirectServoCartesian extends RoboticsAPIApplication
{
    private LBR _lbr;
  
    
    public static double EEfServoPos[];

    public static boolean loopFlag;

    public static double jDispMax[];
    public static double updateCycleJointPos[];
    private UDPServer udpServer;
    int _port;
    
    ServerSocket ss;
    Socket soc;
    


    
    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);
        EEfServoPos=new double[6];
        for(int i=0;i<6;i++)
        {
        	EEfServoPos[i]=0;
        }
        loopFlag=true;
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    private void moveToInitialPosition()
    {
        _lbr.move(
        		ptp(0., Math.PI / 180 * 30.,0.,-Math.PI / 180 * 80.,0.,Math.PI / 180 * 70., 0.).setJointVelocityRel(0.15));
        // Bug fix (the bug happened only in 14R820)
        // Hard-code the Cartesian home position.
        Frame daframe= new Frame(575.87,0.05,397.56,Math.PI,0,Math.PI);
        _lbr.move(
        		lin(daframe).setJointVelocityRel(0.15));
        
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
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
        directServoStartCartezian();
        
    }
    

	private void directServoStartCartezian() {
		
		boolean doDebugPrints = false;

        DirectServo aDirectServoMotion = new DirectServo(
                _lbr.getCurrentJointPosition());

        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting DirectServo motion in position control mode");
        _lbr.moveAsync(aDirectServoMotion);

        getLogger().info("Get the runtime of the DirectServo motion");
        IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion
                .getRuntime();

        Frame aFrame = theDirectServoRuntime.getCurrentCartesianDestination(_lbr.getFlange());
        Frame destFrame = aFrame.copyWithRedundancy();
        // Initiate the initial position 
        EEfServoPos[0]=aFrame.getX();
        EEfServoPos[1]=aFrame.getY();
        EEfServoPos[2]=aFrame.getZ();
        EEfServoPos[3]=aFrame.getAlphaRad();
        EEfServoPos[4]=aFrame.getBetaRad();
        EEfServoPos[5]=aFrame.getGammaRad();
        
        try
        {
            // do a cyclic loop
            // Do some timing...
            // in nanosec

			while(loopFlag==true)
			{

                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system
                theDirectServoRuntime.updateWithRealtimeSystem();
                Frame msrPose = theDirectServoRuntime
                        .getCurrentCartesianDestination(_lbr.getFlange());

                if (doDebugPrints)
                {
                	getLogger().info("Current cartesian goal " + aFrame);
                    getLogger().info("Current joint destination "
                            + theDirectServoRuntime.getCurrentJointDestination());
                }

                Thread.sleep(1);

                for(int kk=0;kk<6;kk++)
                {
                	EEfServoPos[kk]=udpServer.EEFpos[kk];
                }
                // update Cartesian positions
                destFrame.setX(EEfServoPos[0]);
                destFrame.setY(EEfServoPos[1]);
                destFrame.setZ(EEfServoPos[2]);
                destFrame.setAlphaRad(EEfServoPos[3]);
                destFrame.setBetaRad(EEfServoPos[4]);
                destFrame.setGammaRad(EEfServoPos[5]);

                if (doDebugPrints)
                {
                    getLogger().info("New cartesian goal " + destFrame);
                    getLogger().info("LBR position "
	                            + _lbr.getCurrentCartesianPosition(_lbr
	                                    .getFlange()));
                    getLogger().info("Measured cartesian pose from runtime "
	                            + msrPose);
                }

	                theDirectServoRuntime.setDestination(destFrame);
                
            }
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
            //Print statistics and parameters of the motion
            getLogger().info("Simple Cartesian Test \n" + theDirectServoRuntime.toString());

            getLogger().info("Stop the DirectServo motion");
            
        }
        theDirectServoRuntime.stopMotion();
        getLogger().info("Stop the DirectServo motion for the stop instruction was sent");
		
	}


    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        SimulinkIIWADirectServoCartesian app = new SimulinkIIWADirectServoCartesian();
        app.runApplication();
    }
 
    public class UDPServer implements Runnable{
    	
    	public double[] EEFpos={575.87,0.05,397.56,Math.PI,0,Math.PI};
    	
    	double stime=0;
		double endtime=0;
		
    	int _port;
    	int vectorSize=6;
    	int packetCounter=0;
    	
    	byte[] buffer=new byte[8*7];
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
    						int index=0;
    						while(counter <8*vectorSize)
    						{
	    						for(int i=0;i<8;i++)
	    						{
	    							daB[7-i]=buffer[counter];
	    							counter=counter+1;
	    						}
	    						EEFpos[index]=bytesToDouble(daB);
	    						index=index+1;	    						
	    						//System.out.println(bytesToDouble(daB));
    						}
    					}
    				}
    			} catch (SocketException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    				//System.out.println(e.toString());
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
   				SimulinkIIWADirectServoCartesian.loopFlag=false; //Turn off main loop 	
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
