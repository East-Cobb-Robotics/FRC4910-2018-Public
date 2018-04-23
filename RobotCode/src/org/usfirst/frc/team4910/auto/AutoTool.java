package org.usfirst.frc.team4910.auto;

import org.usfirst.frc.team4910.subsystems.DriveTrain;

/**
 * This class is a set of small tools that alow the auto code to look better
 *
 */
public class AutoTool 
{
	public static void abortAuto(DriveTrain drive)
	{
		drive.driveVBus(0, 0);
		System.out.println("[WARNING] Auto aborted");
	}
			
}
