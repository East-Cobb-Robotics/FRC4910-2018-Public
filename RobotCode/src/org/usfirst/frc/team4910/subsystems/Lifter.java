package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterator.Iterate;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.util.LimitSwitch;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lifter {
	
	public static Lifter instance = new Lifter();
	
	public static Lifter getInstance(){
		return instance == null ? instance = new Lifter() : instance;
	}
	
	public enum LiftState{
		Lifting, LiftHeight, Climbing, Locked
	}
	
	private LiftState state;
	
	private TalonSRX liftMotorMaster, liftMotorFollower;
	
	private LimitSwitch topLimit;
	private LimitSwitch bottomLimit;
	
	private boolean topLimitReached;
	private boolean bottomLimitReached;
	
	private DoubleSolenoid climb;
	
	private boolean lowGear = false;;
	
	private Lifter(){
		
		liftMotorMaster = new TalonSRX(RobotMap.lifterMasterPort);
		liftMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		liftMotorMaster.set(ControlMode.PercentOutput, 0);
		liftMotorMaster.setInverted(false);
		liftMotorMaster.setSensorPhase(false);
		liftMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, RobotMap.statusFrameTime, RobotMap.canTimout);
		liftMotorMaster.setNeutralMode(NeutralMode.Brake);
		
		liftMotorFollower = new TalonSRX(RobotMap.lifterFollowerPort);
		liftMotorFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, RobotMap.statusFrameTime, RobotMap.canTimout);
		liftMotorFollower.set(ControlMode.Follower, liftMotorMaster.getDeviceID());
		loadPIDGains();
		
		
		topLimit = new LimitSwitch(RobotMap.topLimitNC, RobotMap.topLimitNO);
		bottomLimit = new LimitSwitch(RobotMap.bottomLimitNC, RobotMap.bottomLimitNO);
		
		climb = new DoubleSolenoid(RobotMap.climbForward, RobotMap.climbReverse);
		
		state = LiftState.Locked;
		
	}
	
	//TODO: Add Shuffleboard publishing of Limit Switch data
	
	private final Iterate LifterLoop = new Iterate(){

		@Override
		public void init() {
			synchronized(Lifter.this){
				liftVBus(0);
				climb.set(DoubleSolenoid.Value.kReverse);
				lowGear = false;
			}
		}
		@Override
		public void run() {
			synchronized(Lifter.this){
				topLimitReached = topLimit.getStatus() == LimitSwitch.State.Active;
				bottomLimitReached = bottomLimit.getStatus() == LimitSwitch.State.Active;
				
				
				if(topLimit.getStatus() == LimitSwitch.State.InvalidTrue){
					//System.err.println("CHECK TOP LIMIT WIRING");
				}
				else if(topLimit.getStatus() == LimitSwitch.State.InvalidFalse){
					System.err.println("FALTY TOP LIMIT");
				}
				
				if(bottomLimit.getStatus() == LimitSwitch.State.InvalidTrue){
					//System.err.println("CHECK BOTTOM LIMIT WIRING");
				}
				else if(bottomLimit.getStatus() == LimitSwitch.State.InvalidFalse){
					System.err.println("FAILTY BOTTOM LIMIT");
				}
				
				switch(state){
				case Lifting:
					break;
				case LiftHeight:
					break;
				case Climbing:
					break;
				case Locked:
					break;
				default:
					System.out.println("Invalid Lifter State: " + state);
					break;
				}
				
			}
		}		@Override
		public void end() {
			liftVBus(0);
		}
		
	};
	
	public Iterate getIterate(){
		return LifterLoop;
	}
	
	public LiftState getState(){
		return state;
	}

	public synchronized void liftVBus(double vbus){
		if(topLimitReached && vbus > 0){
			vbus = 0;
			System.err.println("TOP LIMIT REACHED");
		} 
		else if (bottomLimitReached && vbus < 0)
		{
			vbus = 0;
			System.err.println("BOTTOM LIMIT REACHED");
			
		}
		
		
			if(climb.get() == DoubleSolenoid.Value.kReverse){
				if(state != LiftState.Lifting){
					System.out.println("Lifter State: " + state + " to Lifting");
				}
				state = LiftState.Lifting;
			}
			else{
				if(state != LiftState.Climbing){
					System.out.println("Lifter State: " + state + " to Climbing");
				}
				state = LiftState.Climbing;
			}

			liftMotorMaster.set(ControlMode.PercentOutput, vbus);
		
		
	}
	
	public synchronized void climbToggle(){
		if(climb.get() == DoubleSolenoid.Value.kForward){
			climb.set(DoubleSolenoid.Value.kReverse);
			lowGear = false;
		}
		else{
			climb.set(DoubleSolenoid.Value.kForward);
			lowGear = true;
		}
	}
	
	public synchronized void lockLifter(){
		if(state != LiftState.Locked){
			state = LiftState.Locked;
			liftMotorMaster.selectProfileSlot(0, 0);
			liftMotorMaster.set(ControlMode.MotionMagic, getPosition());
		}
	}
	
	public synchronized void reset(){
		liftMotorMaster.setSelectedSensorPosition(0, 0, RobotMap.canTimout);
	}
	
	public synchronized void liftToHeight(double nativeUnits){
		
		if(state != LiftState.LiftHeight){
			state = LiftState.LiftHeight;
		}
		
		liftMotorMaster.set(ControlMode.MotionMagic, 0);
		liftMotorMaster.configNominalOutputForward(0, RobotMap.canTimout);
		liftMotorMaster.configNominalOutputReverse(0, RobotMap.canTimout);
		
		if(nativeUnits < liftMotorMaster.getSelectedSensorPosition(0)){
			liftMotorMaster.selectProfileSlot(1, 0);
		}
		else{
			liftMotorMaster.selectProfileSlot(0, 0);
		}
		liftMotorMaster.set(ControlMode.MotionMagic, nativeUnits);
		
	}
	
	/**
	 * Returns if the lifter is at a position.
	 * @param position in native units
	 * @return true if it's at that position, false
	 */
	public synchronized boolean atPosition(int position)
	{
		return ((position > (getPosition()-200)) 
		&& (position < (getPosition() + 200)));
	}
	
	public synchronized void liftDisable(){
		liftMotorMaster.set(ControlMode.PercentOutput, 0);
	}
	
	private synchronized void loadPIDGains(){
		liftMotorMaster.config_kP(0, RobotMap.lifterUpKp, RobotMap.canTimout);
		liftMotorMaster.config_kI(0, RobotMap.lifterUpKi, RobotMap.canTimout);
		liftMotorMaster.config_kD(0, RobotMap.lifterUpKd, RobotMap.canTimout);
		liftMotorMaster.config_kF(0, RobotMap.lifterUpKf, RobotMap.canTimout);
		liftMotorMaster.config_IntegralZone(0, RobotMap.lifterUpKIZone, RobotMap.canTimout);
		liftMotorMaster.configMotionCruiseVelocity(RobotMap.lifterUpCruiseVel, RobotMap.canTimout);
		liftMotorMaster.configMotionAcceleration(RobotMap.lifterUpCruiseAcc, RobotMap.canTimout);
		
		liftMotorMaster.config_kP(1, RobotMap.lifterDownKp, RobotMap.canTimout);
		liftMotorMaster.config_kI(1, RobotMap.lifterDownKi, RobotMap.canTimout);
		liftMotorMaster.config_kD(1, RobotMap.lifterDownKd, RobotMap.canTimout);
		liftMotorMaster.config_kF(1, RobotMap.lifterDownKf, RobotMap.canTimout);
		liftMotorMaster.config_IntegralZone(1, RobotMap.lifterDownKIZone, RobotMap.canTimout);
	}
	
	public synchronized int getPosition(){
		return liftMotorMaster.getSelectedSensorPosition(0);
	}
	
	public synchronized boolean getGear() 
	{
		return lowGear;
	}
	
	public synchronized boolean isHighLift()
	{
		return (getPosition() > (RobotMap.SCALE_DOWN - 100));
	}

}
