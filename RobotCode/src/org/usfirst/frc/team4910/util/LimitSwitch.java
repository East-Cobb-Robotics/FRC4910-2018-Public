package org.usfirst.frc.team4910.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
	
	private static int kNCPort, kNOPort;
	
	private DigitalInput limitNC, limitNO;
	
	private State state;
	
	public enum State{
		Inactive, Active, InvalidFalse, InvalidTrue
	}

	public LimitSwitch(int NCPort, int NOPort){
		kNCPort = NCPort;
		kNOPort = NOPort;
		limitNC = new DigitalInput(kNCPort);
		limitNO = new DigitalInput(kNOPort);
		state = getStatus();
	}
	
	public State getStatus(){
		if(limitNC.get() == limitNO.get()){
			if(limitNC.get() == false){
				state = State.InvalidFalse;
			}
			else{
				state = State.InvalidTrue;
			}
		}
		else{
			if(limitNC.get() == false){
				state = State.Inactive;
			}
			else{
				state = State.Active;
			}
		}
		return state;
	}
}
