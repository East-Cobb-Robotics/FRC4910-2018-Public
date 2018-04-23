package org.usfirst.frc.team4910.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team4910.robot.RobotMap;

import java.io.File;

import org.usfirst.frc.team4910.iterator.Iterate;
import org.usfirst.frc.team4910.util.Units;
import org.usfirst.frc.team4910.util.path.PathGenerator;
import org.usfirst.frc.team4910.util.path.PathGenerator.Path;
import org.usfirst.frc.team4910.util.SynchronousPID;

/**
 * The base drive train class for the 2018 robot. It contains all functions
 * required to move for both autonomous and tellop periods of the game.
 * @author Team 4910
 *
 */
public class DriveTrain {
	
	// This is the instance of the robot that is running on the Robot
	public static DriveTrain instance = new DriveTrain();
		
	/**
	 * Contains all the possible states for the driev train
	 * @author Team 4910
	 */
	public enum DriveState 
	{
		Vbus,
		Position, 
		Turn_To_Heading,
		Path_Folowing
	}
	
	/**
	 * The ramp modes supported by DriveTain
	 */
	public enum RampMode
	{
		None,
		LowLift,
		HighLift
	}
	
	/**	 * The instance of DriveTrain
	 * @return The current instance of DriveTrain
	 */
	public static DriveTrain getInstance()
	{
		return instance;
	}
	
	// State
	
	private DriveState state;
	
	// Motors
	private TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;
	
	// Shifter
	private DoubleSolenoid shifter;
	private boolean shiftState, wantedShift;
	
	private AHRS navx;
	
	// GyroPID 
	private SynchronousPID gyroPID;
	private long currentTime;
	private long lastTime;
	private boolean isInErrorZone = false;
	private boolean rightTurn = false;
	private double gyroErrorTolerance = RobotMap.gyroErrorTolerance;
	
	private boolean doneWithMove;
	private int left_move;
	private int right_move;
	
	private double drive_error;
	private double prev_distance;
	
	// Path
	private Trajectory rightPath;
	private Trajectory leftPath;
	
	private DistanceFollower rightFollower;
	private DistanceFollower leftFollower;
	
	
	/**
	 * The default constructor starts the drive train and sets it up to be in 
	 * Vbus mode
	 */
	private DriveTrain() {
		// Configure Left Master CANTalon
		leftMaster  = new TalonSRX(RobotMap.leftMasterPort);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, RobotMap.statusFrameTime, RobotMap.canTimout);
		leftMaster.set(ControlMode.PercentOutput, 0);
		
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotMap.canTimout);
		leftMaster.setInverted(false);
		leftMaster.setSensorPhase(false);
		leftMaster.configContinuousCurrentLimit(RobotMap.driveContinueousMax, RobotMap.canTimout);
		//leftMaster.configPeakCurrentLimit(RobotMap.drivePeckMax, 10);
		//leftMaster.configPeakCurrentDuration(RobotMap.drivePeckTime, 10);
		
		// Configure Right Master CANTalon
		rightMaster = new TalonSRX(RobotMap.rightMasterPort);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, RobotMap.statusFrameTime, RobotMap.canTimout);
		rightMaster.set(ControlMode.PercentOutput, 0);
		
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotMap.canTimout);
		rightMaster.setInverted(true);
		rightMaster.setSensorPhase(false);
		rightMaster.configContinuousCurrentLimit(RobotMap.driveContinueousMax, RobotMap.canTimout);
		//rightMaster.configPeakCurrentLimit(RobotMap.drivePeckMax, 10);
		//rightMaster.configPeakCurrentDuration(RobotMap.drivePeckTime, 10);
		
		// Configure Left Slave CANTalon
		leftSlave   = new TalonSRX(RobotMap.leftSlavePort);
		leftSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, RobotMap.statusFrameTime, RobotMap.canTimout);
		leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
		leftSlave.setInverted(false);
		
		// Configure Right Slave CANTalon
		rightSlave  = new TalonSRX(RobotMap.rightSlavePort);
		rightSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, RobotMap.statusFrameTime, RobotMap.canTimout);
		rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());
		rightSlave.setInverted(true);
		
		loadGains();
		configGyroPID();
		configPathDrive();
		
		// Configure Shifters
		shifter = new DoubleSolenoid(RobotMap.shiftForward, RobotMap.shiftReverse);
		shiftState = false;
		wantedShift = shiftState;
		
		state = DriveState.Vbus;
		
		navx = new AHRS(SerialPort.Port.kUSB);
		setBrakeMode(true);
		
		SmartDashboard.putBoolean("isInErrorZone", isInErrorZone);
	} 
	
	/**
	 * Handles the periodic function for the drive train
	 */
	private final Iterate DriveLoop = new Iterate() {
		
		/**
		 * Handles any enabling tasks for the drive train.
		 */
		public void init()
		{
			synchronized (DriveTrain.this)
			{
				driveVBus(0, 0);
				setBrakeMode(true);
				resetError();
			}
		}
		
		/**
		 * The function that handles any periodic tasks that the drive needs.
		 */
		public void run()
		{
			synchronized (DriveTrain.this)
			{
				switch(state) 
				{
				case Vbus:
					break;
				case Position:
					doneWithMove = doneWithPositionMove();
					if(!doneWithMove)
					{
						updateError();
					}
					break;
				case Turn_To_Heading:
					if (updateTurnToHeading())
					{
						System.out.println("Finished Turn");
						doneWithMove = true;
						driveVBus(0, 0);
						Timer.delay(0.3);
						break;
					}
					else
					{
						doneWithMove = false;
					}
					break;
				case Path_Folowing:
					if(!doneWithMove)
					{
						followPath();
					}
					break;
				default:
					System.out.println("Invalid drive state: " + state);
					break;
				}
				
				if(shiftState){
					shifter.set(DoubleSolenoid.Value.kForward);
				}
				else{
					shifter.set(DoubleSolenoid.Value.kReverse);
				}
				if(wantedShift != shiftState){
					System.out.println("Shifting from " + (shiftState ? "low gear " : "high gear ") + "to " + (wantedShift ? "low gear." : "high gear."));
					shiftState = wantedShift;
				}
				
				SmartDashboard.putData("NavX Heading", navx);
				
			}
		}
		
		/**
		 * Handles any tasks for the drive train on disableing
		 */
		public void end()
		{
			driveVBus(0, 0); 
			
		}
	};
	
	/**
	 * Returns the Iterate Drive Loop
	 * @return The Drive Loop
	 */
	public Iterate getIterate() {return DriveLoop;}	
	
	/**
	 * 
	 * @return The current heading from 1 to 360
	 */
	public double getNavXHeading() {
		return navx.getAngle();
	}
	
	public void resetNavX()
	{
		navx.reset();
	}
	
	/**
	 * Set all the CANTalons breakMode
	 * @param mode the mode to set the break on the CANTalons to
	 */
	public synchronized void setBrakeMode(boolean mode)
	{
		if(mode)
		{
			rightMaster.setNeutralMode(NeutralMode.Brake);
			leftMaster.setNeutralMode(NeutralMode.Brake);
			rightSlave.setNeutralMode(NeutralMode.Brake);
			leftSlave.setNeutralMode(NeutralMode.Brake);
		} else {
			rightMaster.setNeutralMode(NeutralMode.Coast);
			leftMaster.setNeutralMode(NeutralMode.Coast);
			rightSlave.setNeutralMode(NeutralMode.Coast);
			leftSlave.setNeutralMode(NeutralMode.Coast);
		}
	}
	
	
	/**
	 * Drive the robot off of voltage.
	 * This type of driving takes priority over every other 
	 * e.g. if we are driving to a position but this method is called to drive we will stop
	 * going off position and use power
	 * @param left   power for the left drive 0 is now power 1 is full
	 * @param right  power for the right drive 0 is now power 1 is full
	 */
	public synchronized void driveVBus(double left, double right) 
	{
		if(state != DriveState.Vbus)
		{
			leftMaster.configNominalOutputForward(0, RobotMap.canTimout);
			rightMaster.configNominalOutputForward(0, RobotMap.canTimout);

			leftMaster.configNominalOutputReverse(0, RobotMap.canTimout);
			rightMaster.configNominalOutputReverse(0, RobotMap.canTimout);
			setBrakeMode(true);
			state = DriveState.Vbus;
		}
		
		leftMaster.set(ControlMode.PercentOutput, left);
		rightMaster.set(ControlMode.PercentOutput, right);
	}
	
	/**
	 * Commands the each side of the drive train and drive a given
	 * distance in inches using Motion Profiling
	 * @param left_inches how far the left drive train moves
	 * @param right_inches how far the right drive train moves
	 */
	public synchronized void drivePosition(double left_inches, double right_inches)
	{
		if(state != DriveState.Position)
		{
			leftMaster.set(ControlMode.MotionMagic, RobotMap.canTimout);
			rightMaster.set(ControlMode.MotionMagic, RobotMap.canTimout);
			
			leftMaster.configNominalOutputForward(0, RobotMap.canTimout);
			rightMaster.configNominalOutputForward(0, RobotMap.canTimout);

			leftMaster.configNominalOutputReverse(0, RobotMap.canTimout);
			rightMaster.configNominalOutputReverse(0, RobotMap.canTimout);
			setBrakeMode(true);
			state = DriveState.Position;
		}
		if(!shiftState)
		{
			leftMaster.selectProfileSlot(0, 0);
			rightMaster.selectProfileSlot(0, 0);
		}
		else
		{
			leftMaster.selectProfileSlot(1, 0);
			rightMaster.selectProfileSlot(1, 0);
		}
		
		resetEncoders();
		resetError();
		right_move = Units.inchesToNativeUnits(right_inches);
		left_move  = Units.inchesToNativeUnits(left_inches);
		doneWithMove = false;
		leftMaster.set(ControlMode.MotionMagic, Units.inchesToNativeUnits(left_inches));
		rightMaster.set(ControlMode.MotionMagic, Units.inchesToNativeUnits(right_inches));
	}
	
	/**
	 * Sets the cruise speed of the talons
	 * @param speed in in/sec
	 */
	public synchronized void setSpeed(double speed)
	{
		leftMaster.configMotionCruiseVelocity((int)Units.inchesPerSecToNativeUinitsPerHunderedMilSec(speed), RobotMap.canTimout);
		rightMaster.configMotionCruiseVelocity((int)Units.inchesPerSecToNativeUinitsPerHunderedMilSec(speed), RobotMap.canTimout);
	}
	
	/**
	 * Checks to see if the encoder count is eqal to the target encoder count
	 * The TalonSRXs do not have a method to tell us if they are done
	 * with a MotionMagic move so this round about method must be used
	 * @return
	 */
	private synchronized boolean doneWithPositionMove()
	{
		boolean rightDone = ((right_move > (rightMaster.getSelectedSensorPosition(0)-100)) && (right_move < rightMaster.getSelectedSensorPosition(0) + 100));
		boolean leftDone = ((left_move > (leftMaster.getSelectedSensorPosition(0)-100)) && (left_move < leftMaster.getSelectedSensorPosition(0) + 100));
		return (leftDone && rightDone);
	}
	
	/**
	 * Returns if we are at a specific position in a position move
	 * @param inches The position to check for
	 * @return true if there false if not
	 */
	public synchronized boolean doneAtPosition(double inches)
	{
		int move = Units.inchesToNativeUnits(inches);
		boolean rightDone = ((move > (rightMaster.getSelectedSensorPosition(0)-100)) && (move < rightMaster.getSelectedSensorPosition(0) + 100));
		boolean leftDone = ((move > (leftMaster.getSelectedSensorPosition(0)-100)) && (move < leftMaster.getSelectedSensorPosition(0) + 100));
		return (leftDone && rightDone);
	}
	
	/**
	 * Returns if the drive train is done with the current move
	 * @return if we are done with the last closed-loop given drive command.
	 */
	public boolean doneWithCurrentMove()
	{
		return doneWithMove;
	}
	
	/**
	 * Reset Encoder positions to 0
	 */
	public synchronized void resetEncoders()
	{
		leftMaster.setSelectedSensorPosition(0, 0, RobotMap.canTimout);
		rightMaster.setSelectedSensorPosition(0, 0, RobotMap.canTimout);
	}
	
	/**
	 * Loads the PID gains onto the TalonSRXs
	 */
	private synchronized void loadGains()
	{
		leftMaster.config_kP(0, RobotMap.highGearKp, RobotMap.canTimout);
		leftMaster.config_kD(0, RobotMap.highGearKd, RobotMap.canTimout);
		leftMaster.config_kI(0, RobotMap.highGearKi, RobotMap.canTimout);
		leftMaster.config_kF(0, RobotMap.highGearKf, RobotMap.canTimout);
		leftMaster.config_IntegralZone(0, RobotMap.highGearkIzone, RobotMap.canTimout);
		leftMaster.configMotionCruiseVelocity((int)Units.inchesPerSecToNativeUinitsPerHunderedMilSec(RobotMap.highGearMaxVelocity), 0);
		leftMaster.configMotionAcceleration(RobotMap.highGearMaxAccel, RobotMap.canTimout);
		
		
		rightMaster.config_kP(0, RobotMap.highGearKp, RobotMap.canTimout);
		rightMaster.config_kD(0, RobotMap.highGearKd, RobotMap.canTimout);
		rightMaster.config_kI(0, RobotMap.highGearKi, RobotMap.canTimout);
		rightMaster.config_kF(0, RobotMap.highGearKf, RobotMap.canTimout);
		rightMaster.config_IntegralZone(0, RobotMap.highGearkIzone, RobotMap.canTimout);
		rightMaster.configMotionCruiseVelocity((int)Units.inchesPerSecToNativeUinitsPerHunderedMilSec(RobotMap.highGearMaxVelocity), 0);
		rightMaster.configMotionAcceleration(RobotMap.highGearMaxAccel, 0);
		
		leftMaster.config_kP(1, RobotMap.lowGearKp, RobotMap.canTimout);
		leftMaster.config_kD(1,  RobotMap.lowGearKd, RobotMap.canTimout);
		leftMaster.config_kI(1, RobotMap.lowGearKi, RobotMap.canTimout);
		leftMaster.config_IntegralZone(1, RobotMap.lowGearIzone, RobotMap.canTimout);
		
		rightMaster.config_kP(1, RobotMap.lowGearKp, RobotMap.canTimout);
		rightMaster.config_kD(1,  RobotMap.lowGearKd, RobotMap.canTimout);
		rightMaster.config_kI(1, RobotMap.lowGearKi, RobotMap.canTimout);
		rightMaster.config_IntegralZone(1, RobotMap.lowGearIzone, RobotMap.canTimout);
	}
	
	/**
	 * Returns the state of the drive train
	 * @return The current dive state.
	 */
	public DriveState getState(){
		return state;
	}
	
	/**
	 * Cycles the gearing.
	 */
	public synchronized void shift()
	{
		wantedShift = !wantedShift;
	}

	/**
	 * Returns the current gear state 
	 * @return true if Low Gear, false if High Gear 
 	 */ 
	public boolean getGear()
	{ 
 		return shiftState; 
	} 
	
	/**
	 * Sets drive to high gear
	 */
	public synchronized void setHighGear()
	{
		wantedShift = false;
	}
	
	/**
	 * Sets Ramp Mode
	 * @param ramp the RampMode to go to
	 */
	public synchronized void setRampMode(RampMode ramp)
	{
		switch(ramp) 
		{
		case None:
			leftMaster.configOpenloopRamp(0, RobotMap.canTimout);
			rightMaster.configOpenloopRamp(0, RobotMap.canTimout);
			break;
		case LowLift:
			leftMaster.configOpenloopRamp(0.25, RobotMap.canTimout);
			rightMaster.configOpenloopRamp(0.25, RobotMap.canTimout);
			break;
		case HighLift:
			leftMaster.configOpenloopRamp(1, RobotMap.canTimout);
			rightMaster.configOpenloopRamp(1, RobotMap.canTimout);
			break;
		default:
			System.out.println("[WARNING] Invailed ramp mode receaved");
			return;
		}
		System.out.println("[INFO] Ramp mode set to " + ramp.toString());
	}
	
	/**
	 * Sets drive to low gear
	 */
	public synchronized void setLowGear()
	{
		wantedShift = true;
	}
	
	/**
	 * Sets up the Gyro PID for use.
	 */
	public synchronized void configGyroPID()
	{
		gyroPID = new SynchronousPID(RobotMap.gyroKp,
									RobotMap.gyroKi,
									RobotMap.gyroKd,
									RobotMap.gyroKf);
		gyroPID.setIZoneRange(-RobotMap.gyroIzone, RobotMap.gyroIzone);
		gyroPID.setContinuous(false);
		gyroPID.setInputRange(-360, 360);
		gyroPID.setOutputRange(-0.95, 0.95);
		gyroPID.setTolerance(this.gyroErrorTolerance);
	}
	
	/**
	 * Only should be used when doing PID tuning
	 * Lets the PIDF variables be set to non-default values
	 * @param kP The prepotional gain
	 * @param kI The integral gain
	 * @param kD The derivative gain
	 * @param kF The feed forward value
	 * @param iZone the zone where integral gain is used
	 */
	public synchronized void setGyroPID(double kP, double kI, double kD, double kF, double iZone)
	{
		gyroPID.setPIDF(kP, kI, kD, kF);
		gyroPID.setIZoneRange(-iZone, iZone);
	}
	
	/**
	 * Commands the Drive Train to Turn to a heading
	 * @param heading the heading to turn to
	 */
	public synchronized void turnToHeading(double heading)
	{
		try 
		{
			gyroPID.setSetpoint(heading);
			setBrakeMode(true);
			state = DriveState.Turn_To_Heading;
			doneWithMove = false;
			this.gyroErrorTolerance = RobotMap.gyroErrorTolerance;
			gyroPID.setTolerance(this.gyroErrorTolerance);
			Timer.delay(0.01);
			if((heading - getNavXHeading()) > 0) 
			{
				rightTurn = true;
			}
			else
			{
				rightTurn = false;
			}
			updateTurnToHeading();
		} 
		catch (NullPointerException e)
		{
			System.out.println("ERROR GYRO PID not initalized: " + e);
			throw e;
		}
	}
	
	/**
	 * Starts a turn to head move and specifies the drive train to turn with.
	 * @param heading the heading to turn to 
	 * @param right_turn wheather to use the right drive or the left drive
	 */
	public synchronized void turnToHeading(double heading, boolean right_turn)
	{
		try 
		{
			gyroPID.setSetpoint(heading);
			setBrakeMode(true);
			state = DriveState.Turn_To_Heading;
			doneWithMove = false;
			this.gyroErrorTolerance = RobotMap.gyroErrorTolerance;
			gyroPID.setTolerance(this.gyroErrorTolerance);
			Timer.delay(0.01);
			rightTurn = right_turn;
		} 
		catch (NullPointerException e)
		{
			System.out.println("ERROR: GYRO PID not initalized: " + e);
			throw e;
		}
	}
	
	/**
	 * Commands the Drive Train to Turn to a heading
	 * @param heading the heading to turn to
	 * @param tolerance the amount of allowable error in the turn
	 */
	public synchronized void turnToHeading(double heading, double tolerance)
	{
		try 
		{
			gyroPID.setSetpoint(heading);
			setBrakeMode(true);
			state = DriveState.Turn_To_Heading;
			doneWithMove = false;
			this.gyroErrorTolerance = tolerance;
			gyroPID.setTolerance(tolerance);
			Timer.delay(0.01);
			if((heading - getNavXHeading()) > 0) 
			{
				rightTurn = true;
			}
			else
			{
				rightTurn = false;
			}
			updateTurnToHeading();
		} 
		catch (NullPointerException e)
		{
			System.out.println("ERROR GYRO PID not initalized: " + e);
			throw e;
		}
	}
	
	/**
	 * Starts a turn to head move and specifies the drive train to turn with.
	 * @param heading the heading to turn to 
	 * @param tolerance the allowable error in the turn
	 * @param right_turn wheather to use the right drive or the left drive
	 */
	public synchronized void turnToHeading(double heading, double tolerance, boolean right_turn)
	{
		try 
		{
			gyroPID.setSetpoint(heading);
			setBrakeMode(true);
			state = DriveState.Turn_To_Heading;
			doneWithMove = false;
			this.gyroErrorTolerance = tolerance;
			gyroPID.setTolerance(tolerance);
			Timer.delay(0.01);
			rightTurn = right_turn;
		} 
		catch (NullPointerException e)
		{
			System.out.println("ERROR: GYRO PID not initalized: " + e);
			throw e;
		}
	}
	
	/**
	 * The periodic function called during a turn to heading move
	 * @return true if turn is done, false if it not done
	 */
	private synchronized boolean updateTurnToHeading()
	{
		double turn = gyroPID.calculate(-getNavXHeading());
		
		// If we are in the error tolerance zone.
		if((gyroPID.getError() < this.gyroErrorTolerance) && (gyroPID.getError() > -this.gyroErrorTolerance))
		{
			// Is did we just enter the error zone?
			if(isInErrorZone == false)
			{
				// Our last time is now (because we just entered the loop)
				lastTime = System.currentTimeMillis();
			}
			// Grab our current time
			currentTime = System.currentTimeMillis();
			this.isInErrorZone = true; // we are in the tolerance zone
			
			if(Math.abs(currentTime - lastTime) > RobotMap.gyroTolernaceZoneTime)
			{
				SmartDashboard.putBoolean("isInErrorZone", isInErrorZone);
				return true; // We are done with the turn
			}
		}
		else
		{
			isInErrorZone = false;
		}
		if(rightTurn)
		{
			rightMaster.set(ControlMode.PercentOutput, -turn);
			resetEncoders();
			leftMaster.set(ControlMode.MotionMagic, 0);
		}
		else
		{
			leftMaster.set(ControlMode.PercentOutput, turn);
			resetEncoders();
			rightMaster.set(ControlMode.MotionMagic, 0);
		}
		
		SmartDashboard.putBoolean("isInErrorZone", isInErrorZone);
		return false; // We are not done 
	}
	
	/**
	 * Returns the navx object
	 * @return the navx object
	 */
	public synchronized AHRS getNavXObj(){
		return navx;
	}

	/**
	 * Updates the heading the gyro is turing to
	 * @param heading the absolute heading
	 */
	public synchronized void updateTurnToHeadingSetpoint(double heading)
	{
		gyroPID.setSetpoint(heading);
	}
	
	/**
	 * Resets the error calc for strait moves
	 */
	public synchronized void resetError()
	{
		drive_error = 0;
		prev_distance = 0;
	}
	
	/**
	 * Returns the current drive error
	 * @return the current drive error
	 */
	public synchronized double getError()
	{
		return drive_error;
	}
	
	/**
	 * Returns the left encoder position in inches
	 * @return left encoder position in inches
	 */
	public synchronized double getEncoderPos() {
		return Units.nativeUnitsToInches(leftMaster.getSelectedSensorPosition(0));
	}
	
	/**
	 * Keeps track of the error over a position move
	 * Must be called at a rate of at least 20ms.
	 * The faster the call rate the more accurate the 
	 * calculation is.
	 */
	private synchronized void updateError()
	{
		double distance = Units.nativeUnitsToInches((rightMaster.getSelectedSensorPosition(0) + leftMaster.getSelectedSensorPosition(0)/2));
		double delta_distance = Math.abs(distance - prev_distance);
		double heading_error = gyroPID.getSetpoint() - getNavXHeading();
		
		drive_error += delta_distance*Math.sin(heading_error*(Math.PI/180));
		
		prev_distance = distance;
	}
	
	/**
	 * Configers the drive train to follow path and then has it start flowing
	 * @param path the path to follow
	 */
	public synchronized void startFollowPath(Path path)
	{
		if(!PathGenerator.isPathGenerated(path))
		{
			System.out.println("[ERROR] Path folowing aborted. Requested path " + path + " not found\\generated");
			return;
		}
		
		resetEncoders();
		doneWithMove = false;
		
		File file_right = new File(path.toString() + PathGenerator.rightEnd);
		File file_left = new File(path.toString() + PathGenerator.leftEnd);
		
		leftPath  = Pathfinder.readFromCSV(file_left);
		rightPath = Pathfinder.readFromCSV(file_right); 
		
		leftFollower.reset();
		rightFollower.reset();
		
		leftFollower.setTrajectory(leftPath);
		rightFollower.setTrajectory(rightPath);
		
		state = DriveState.Path_Folowing;
		
		followPath();
	}
	
	/**
	 * Configures the drive train to follow a forward or backwards path and start the
	 * path following
	 * @param path The path to follow
	 * @param invert true: drive the path backwards false: drive the path forwords
	 */
	public synchronized void startFollowPath(Path path, boolean invert)
	{
		if(!PathGenerator.isPathGenerated(path))
		{
			System.out.println("[ERROR] Path folowing aborted. Requested path " + path + " not found\\generated");
			return;
		}
		
		resetEncoders();
		doneWithMove = false;
		
		File file_right = new File(path.toString() + PathGenerator.rightEnd);
		File file_left = new File(path.toString() + PathGenerator.leftEnd);
		
		leftPath  = Pathfinder.readFromCSV(file_left);
		rightPath = Pathfinder.readFromCSV(file_right); 
		
		if(invert)
		{
			leftPath = PathGenerator.invenrtPath(leftPath);
			rightPath = PathGenerator.invenrtPath(rightPath);
			
		}
		
		leftFollower.reset();
		rightFollower.reset();
		
		leftFollower.setTrajectory(leftPath);
		rightFollower.setTrajectory(rightPath);
		
		state = DriveState.Path_Folowing;
		
		followPath();
	}
	
	/**
	 * Periotic fuction called to follow a given path.
	 * @return true of the drive is done otherwise false
	 */
	private synchronized boolean followPath()
	{
		if(leftFollower.isFinished() && rightFollower.isFinished())
		{
			driveVBus(0, 0);
			doneWithMove = true;
			System.out.println("[INFO] Path done.");
			return true;
		}
		double leftPower = leftFollower.calculate(-Units.nativeUnitsToInches(leftMaster.getSelectedSensorPosition(0)));
		double rightPower = rightFollower.calculate(-Units.nativeUnitsToInches(rightMaster.getSelectedSensorPosition(0)));
		double heading = getNavXHeading();
		double wanted_heading = Pathfinder.r2d(leftFollower.getHeading());
		
		double heading_error = Pathfinder.boundHalfDegrees(wanted_heading - heading);
		
		double turn = 0.8 * (-1.0/80.0) * heading_error;
	
		leftMaster.set(ControlMode.PercentOutput, -(leftPower + turn));
		rightMaster.set(ControlMode.PercentOutput, -(rightPower - turn));
		
		return false;
		
	}
	
	/**
	 * Configure the path following algorithms
	 */
	private void configPathDrive()
	{
		leftFollower = new DistanceFollower(leftPath);
		leftFollower.configurePIDVA(
				RobotMap.pathKp,
				RobotMap.pathKi,
				RobotMap.pathKd,
				RobotMap.pathKv, 
				RobotMap.pathKa);
		
		rightFollower = new DistanceFollower(rightPath);
		rightFollower.configurePIDVA(
				RobotMap.pathKp,
				RobotMap.pathKi,
				RobotMap.pathKd,
				RobotMap.pathKv, 
				RobotMap.pathKa);
	}
	
}
