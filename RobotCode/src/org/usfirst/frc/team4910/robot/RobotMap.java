
package org.usfirst.frc.team4910.robot;

/**
 * This class is masking the electrical diagram but in code.
 * It tells us where each component is plugged in so that we can
 * quickly update the entire robot for electrical wiring changes
 */
public class RobotMap {
	
	//CanBUS
	public final static int canTimout = 50;
	public final static int statusFrameTime = 50;
	
	// Drive Train Ports
	public final static int leftMasterPort  = 1;
	public final static int leftSlavePort   = 2;

	public final static int rightMasterPort = 3;
	public final static int rightSlavePort  = 4;
	
	//The Gripper
	public final static int gripperForward = 7;
	public final static int gripperReverse = 0;
	
	//The Articulator (hinge)
	public final static int hingeForward = 6;
	public final static int hingeReverse = 1;
	
	//The Shifters (drive)
	public final static int shiftForward = 4;
	public final static int shiftReverse = 3;
	
	// Drive Train Physical Properties
	public final static double wheelDiamiter = 6.1; // inches
	
	// Drive Train Control Loop 
	
	public final static double highGearKp  = 3.84;
	public final static double highGearKi  = 0;
	public final static double highGearKd  = 19.2;
	public final static double highGearKf  = 1.1108;
	public final static int highGearkIzone = 0;
	public final static double highGearMaxVelocity = 120; // inches per sec
	public final static int highGearMaxAccel = 1728; //Native units per 100 millisec
	
	public final static double lowGearKf = 0;
	public final static double lowGearKp = 0;
	public final static double lowGearKd = 0;
	public final static double lowGearKi = 0;
	public final static int lowGearIzone = 0;
	
	//Gyro PIDF
	public final static double gyroKf = 0; 
	public final static double gyroKp = 0.16; 
	public final static double gyroKd = 1.5; 
	public final static double gyroKi = 0.000; 
	public final static double gyroIzone = 5; 
	public final static double gyroErrorTolerance = 0.65;   
	public final static double gyroTolernaceZoneTime = 10; // msec 
	
	//Drive Power
	public final static int driveContinueousMax = 50;
	public final static int drivePeckMax = 65;
	public final static int drivePeckTime = 500;
	
	// Lifter Ports
	public final static int lifterMasterPort = 8;
	public final static int lifterFollowerPort = 9;

	public final static double lifterUpKp = 1.8;
	public final static double lifterUpKi = 0;
	public final static double lifterUpKd = 20;
	public final static double lifterUpKf = 0.2637;
	public final static int lifterUpKIZone = 0;
	public final static int lifterUpCruiseVel = 5000;//4400 
	public final static int lifterUpCruiseAcc = 5820;
	public final static double lifterDownKp = .125;
	public final static double lifterDownKi = 0;
	public final static double lifterDownKd = 10;
	public final static double lifterDownKf = 0.2637;
	public final static int lifterDownKIZone = 0;
	public final static int lifterDownCruiseVel = 3880;
	public final static int lifterDownCruiseAcc = 11640;
	public static final double liftDiameter = 1;
	
	//The Lifter Shifter (the climber)
	public final static int climbForward = 5;
	public final static int climbReverse = 2;
	
	//The Joysticks
	public final static int leftJoy1Port = 0;
	public final static int rightJoy1Port = 1;
	public final static int leftJoy2Port = 2;
	public final static int rightJoy2Port = 3;
	
	//The Limit Switches
	public final static int bottomLimitNC = 0;
	public final static int bottomLimitNO = 1;
	public final static int topLimitNC = 2;
	public final static int topLimitNO = 3;
	
	// Buttons
	public final static int PIDBack = 3;
	public final static int PIDForoward = 4;
	
	public final static int toggleGrip = 3;
	public final static int toggleHeight = 4;
	public final static int toggleComp = 7;
	public final static int toggleClimb = 2;
	public final static int driveShift = 2;
	public final static int dualStickToggle = 5;
	public final static int dualToggleClimb = 5;
	public final static int lockOverride = 6;
	
	//Lifter Setpoints
	//Comp: 35400, Practice: 34400
	public final static int MAX_SAFE_HEIGHT = 35400;
	public final static int SCALE_DOWN = 30100;
	public final static int SWITCH_FENCE = 12500;
	public final static int CUBE_STACK = 7000;
	//Practice: 200;
	public final static int LIFTER_DOWN = 900;
	public final static int LIFTER_RANGE = 34670;
	
	// Path Config
	public final static double maxAccelPath = 150; // in/sec^2
	public final static double maxJerkPath = 2362; // in/sec^3
	public final static double weelbase = 24.81;
	
	public final static double pathKp = 0.15;
	public final static double pathKi = 0;
	public final static double pathKd = 0.000;
	public final static double pathKv = 1/RobotMap.highGearMaxVelocity;
	public final static double pathKa = 0.005;
}
