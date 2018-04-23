package org.usfirst.frc.team4910.util;

import org.usfirst.frc.team4910.robot.RobotMap;

/**
 * This class is a collection of conversion of various units
 * @author alex
 *
 */
public class Units {

	public static double inchesToRotations(double left_inches)
	{
		return (left_inches / (RobotMap.wheelDiamiter * Math.PI));
	}
	
	public static int rotationsToNativeUnits(double rotations)
	{
		return (int)(4096*rotations);
	}
	
	public static int inchesToNativeUnits(double inches)
	{
		return rotationsToNativeUnits(inchesToRotations(inches));
	}
	
	public static double rpmToNativeUnitsePerHunderedMilSec(double rpm)
	{
		return (rpm * 4096)/600;
	}
	
	public static double nativeUnitsePerHunderedMilSecToRPM(double speed)
	{
		return (speed/4096) * 600;
 	}
	
	public static double rpmToInchesPerSec(double rpm)
	{
		return (rpm * (RobotMap.wheelDiamiter * Math.PI))/60;
	}
	
	public static double nativeUnitesPerHunderedMilSecToInchesPerSec(double speed)
	{
		return rpmToInchesPerSec(nativeUnitsePerHunderedMilSecToRPM(speed));
	}
	
	public static double inchesPerSecToRPM(double inchesPerSec)
	{
		return inchesToRotations(inchesPerSec) * 60;
	}
	
	public static double inchesPerSecToNativeUinitsPerHunderedMilSec(double inches)
	{
		return rpmToNativeUnitsePerHunderedMilSec(inchesPerSecToRPM(inches));
	}
	
	public static double rotationsToInches(double rotations)
	{
		return (RobotMap.wheelDiamiter * Math.PI) * rotations;
	}
	
	public static double nativeUnitsToRoations(double native_units)
	{
		return native_units/4096;
	}
	
	public static double nativeUnitsToInches(double native_units)
	{
		return rotationsToInches(nativeUnitsToRoations(native_units));
	}
	
}
