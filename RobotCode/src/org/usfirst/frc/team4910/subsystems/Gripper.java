package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterator.Iterate;
import org.usfirst.frc.team4910.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * The class that runs our gripper
 * @author Team 4910
 *
 */
public class Gripper {

	public static Gripper instance = new Gripper();
	
	/**
	 *  Contains all possible states the gripper can be in
	 * @author Team 4910
	 *
	 */
	public enum GripState{
		Open, 
		Closed
	}
	
	public static Gripper getInstance(){
		return instance == null ? instance = new Gripper() : instance;
	}
	
	//State
	private GripState state;

	private GripState wantedState;
	
	//Pneumatics
	private DoubleSolenoid gripper;
	private DoubleSolenoid hinge;	
	private Gripper(){
		gripper = new DoubleSolenoid(RobotMap.gripperForward, RobotMap.gripperReverse);
		hinge = new DoubleSolenoid(RobotMap.hingeForward, RobotMap.hingeReverse);
		state = GripState.Open;
		wantedState = state;
	}
	
	private final Iterate GripLoop = new Iterate () {

		@Override
		public void init() {
			synchronized(Gripper.this){
				gripper.set(DoubleSolenoid.Value.kForward);
				state = GripState.Closed;
				liftGripper();
			}
		}

		@Override
		public void run() {
			synchronized(Gripper.this){
				switch(state){
				case Open:
					gripper.set(DoubleSolenoid.Value.kForward);
					break;
				case Closed:
					gripper.set(DoubleSolenoid.Value.kReverse);
					break;
				default:
					System.out.println("WARNING Invalid gripper state: \"" + state + "\"");
					break;
				}
				
				if(wantedState != state){
					System.out.println("Changing gripper state from: " + state + " to " + wantedState);
					state = wantedState;
				}
			}
			
		}

		@Override
		public void end() {
			 
		}
		
	};
	
	public Iterate getIterate(){
		return GripLoop;
	}
	
	public GripState getState(){
		return state;
	}
	
	public synchronized void toggleGrip(){
		if(state.equals(GripState.Open)){
			wantedState = GripState.Closed;
		}
		else{
			wantedState = GripState.Open;
		}
	}
	
	public synchronized void openGripper(){
		wantedState = GripState.Open;
	}
	
	public synchronized void closeGripper(){
		wantedState = GripState.Closed;
	}
	
	public synchronized void liftGripper(){
		hinge.set(DoubleSolenoid.Value.kReverse);
		System.out.println("Lifting gripper @" + Timer.getFPGATimestamp());
	}
	
	public synchronized void lowerGripper(){
		hinge.set(DoubleSolenoid.Value.kForward);
		System.out.println("Lowering gripper @" + Timer.getFPGATimestamp());
	}
	
	public synchronized void toggleHeight(){
		if(hinge.get() == DoubleSolenoid.Value.kForward){
			liftGripper();
		}
		else{
			hinge.set(DoubleSolenoid.Value.kForward);
			lowerGripper();
		}
	}
	
	public boolean getHeight(){
		return hinge.get() == DoubleSolenoid.Value.kReverse ? true : false;
	}
	
	public synchronized void setState(GripState newState){
		wantedState = newState;
	}
	
	
}
