package org.usfirst.frc.team4910.util;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureGauge {

	private AnalogInput gauge;
	
	public PressureGauge() {
		this(0);
	}
	
	public PressureGauge(int port) {
		gauge = new AnalogInput(port);
	}
	
	public double getPressure() {
		double nominalV = 4.8407407;
		double pressure = 250 * (getVoltage()/nominalV) - 25;
		return pressure;
	}
	
	public double getVoltage() {
		return gauge.getVoltage();
	}
}
