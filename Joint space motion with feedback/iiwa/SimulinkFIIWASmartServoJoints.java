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

import lbrExampleApplications.SimulinkFIIWADirectServoJoints.UDPPublisher;
import lbrExampleApplications.SimulinkIIWADirectServoJoints.UDPServer;

/*
 * SimulinkIIWA interface, soft real-time control in joint position mode
 * Copyright: Mohammad SAFEEA, 22nd/May/2018
 * License: MIT license
 * 
 * This is a UDP server that receives motion commands from Simulink.
 * This server works together with the Simulink example: (feedBackSmartDirectServoJoints.slx)
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
 * 1- Add this class to a new project  using kuka's Sunrsie.Workbench.
 * 2- Add reference to the Direct/SmartServo from inside the (StationSetup.cat) file. 
 * 3- Change the following variables according to your preference:
 * 	* externa_PC_IP: change this variable according to the IP of the PC used for control.
 *  * TRANSLATION_OF_TOOL: translation of tool center point with respect to flange frame.
 *  * massOfTool: mass of the tool
 *  * toolsCOMCoordiantes: coordinates of center of mass of tool with regards to the frame of the flange.
 *
 * 
 * Utilization instructions:
 * 1- Synchronize the project to the controller of the robot.
 * 2- Run this application from the teach-pad of the robot,
 *    This application has a time out of 10 seconds, if a connection
 *    is not established during the 10 seconds interval the server will
 *    turn off.
 * 3- Start the Simulink example (feedBackSmartDirectServoJoints.slx)
 *    from the external PC.
 * 4- To change the timeout value, change the argument for the instruction
 *    soc.setSoTimeout(10000),  the argument is in milliseconds.
 * 
 */

public class SimulinkFIIWASmartServoJoints extends RoboticsAPIApplication
{
	private static String externa_PC_IP="172.31.69.55";
    private LBR _lbr;
    private ISmartServoRuntime _theSmartServoRuntime = null;

    // Tool Data
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0.4;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

    private int _count = 0;

    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 1;
    private int _steps = 0;

    public static boolean loopFlag;
    public static double[] jpos;
    private UDPServer udpServer;
    
 
    
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
    			while(loopFlag)
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
    	
 
    
    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);
        jpos=new double[7];
        for(int i=0;i<7;i++)
        {
        	jpos[i]=0;
        }
        loopFlag=true;
		addLoadData();
    }

    private void addLoadData()
    {
    	 // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        _loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        _toolAttachedToLBR = new Tool("Tool", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2]);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(_lbr.getFlange());
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    private void moveToInitialPosition()
    {
        _lbr.move(
        		ptp(0., 0.,0.,0.,0.,0., 0.).setJointVelocityRel(0.15));
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
        new UDPPublisher(externa_PC_IP);
    	startSmartServo();
    }
    
    private void startSmartServo(){
        
        boolean doDebugPrints = false;

        JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());
        SmartServo aSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 20% of systems abilities
        aSmartServoMotion.setJointAccelerationRel(0.2);
        aSmartServoMotion.setJointVelocityRel(0.2);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting SmartServo motion in position control mode");
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(aSmartServoMotion);

        getLogger().info("Get the runtime of the SmartServo motion");
        _theSmartServoRuntime = aSmartServoMotion.getRuntime();

        // create an JointPosition Instance, to play with
        JointPosition destination = new JointPosition(
                _lbr.getJointCount());
        for(int i=0;i<7;i++){
        	jpos[i]=destination.get(i);
        }
        getLogger().info("Start loop");
        // For Roundtrip time measurement...
        StatisticTimer timing = new StatisticTimer();
        try
        {
            // do a cyclic loop
            // Refer to some timing...
            // in nanosec
            
            long startTimeStamp = System.nanoTime();
            while(loopFlag==true)
            {
            	_steps=_steps+1;
                // Timing - draw one step
                OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
                _theSmartServoRuntime.updateWithRealtimeSystem();
                // Get the measured position
                JointPosition curMsrJntPose = _theSmartServoRuntime
                        .getAxisQMsrOnController();

                for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                	jpos[k]=udpServer.jpos[k];
                    destination.set(k, jpos[k]);
                }
                _theSmartServoRuntime.setDestination(destination);

                // ////////////////////////////////////////////////////////////
                if (doDebugPrints)
                {
                    getLogger().info("Step " + _steps + " New Goal "
                            + destination);
                    getLogger().info("Fine ipo finished " + _theSmartServoRuntime.isDestinationReached());
                    if (_theSmartServoRuntime.isDestinationReached())
                    {
                        _count++;
                    }
                    getLogger().info("Ipo state " + _theSmartServoRuntime.getFineIpoState());
                    getLogger().info("Remaining time " + _theSmartServoRuntime.getRemainingTime());
                    getLogger().info("LBR Position "
                            + _lbr.getCurrentJointPosition());
                    getLogger().info(" Measured LBR Position "
                            + curMsrJntPose);
                    if (_steps % 100 == 0)
                    {
                        // Some internal values, which can be displayed
                        getLogger().info("Simple Joint Test - step " + _steps + _theSmartServoRuntime.toString());

                    }
                }
                aStep.end();

            }
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
        }
        ThreadUtil.milliSleep(1000);

        //Print statistics and parameters of the motion
        getLogger().info("Displaying final states after loop ");
        getLogger().info(getClass().getName() + _theSmartServoRuntime.toString());
        // Stop the motion

        getLogger().info("Stop the SmartServo motion");
        _theSmartServoRuntime.stopMotion();

        getLogger().info(_count + " times was the destination reached.");
        getLogger().info("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger().info("Under Windows, you should play with the registry, see the e.g. the SmartServo Class javaDoc for details");
        }
    }

    
    double getTheDisplacment(double dj)
    {
    	
    	return dj;
    	/*
    	double   a=0.07; 
    	double b=a*0.75; 
		double exponenet=-Math.pow(dj/b, 2);
		return Math.signum(dj)*a*(1-Math.exp(exponenet));
		*/
    }


    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        SimulinkFIIWASmartServoJoints app = new SimulinkFIIWASmartServoJoints();
        app.runApplication();
    }
 
    public class UDPServer implements Runnable{
    	
    	public double[] jpos=new double[7];
    	
    	double stime=0;
		double endtime=0;
		
    	int _port;
    	int vectorSize=7;
    	int packetCounter=0;
    	
    	byte[] buffer=new byte[8*vectorSize];
    	UDPServer(int port)
    	{
    		for(int i=0;i<7;i++)
    		{
    			jpos[i]=0;
    		}
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
	    						jpos[jointNum]=bytesToDouble(daB);
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
   				SimulinkFIIWASmartServoJoints.loopFlag=false; //Turn off main loop 	
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
