package org.usfirst.frc.team4910.auto;

import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.subsystems.DriveTrain;
import org.usfirst.frc.team4910.subsystems.Gripper;
import org.usfirst.frc.team4910.subsystems.Lifter;
import org.usfirst.frc.team4910.util.path.PathGenerator;
import org.usfirst.frc.team4910.util.path.PathGenerator.Path;
import org.usfirst.frc.team4910.auto.AutoTool;

import edu.wpi.first.wpilibj.Timer;

public class AutoCommand {
	
	public static void leftCloseSwitchMove(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		lifter.liftToHeight(RobotMap.SWITCH_FENCE);
		/*
		drive.drivePosition(-108, -108);
		double error = drive.getError();
		System.out.println("[INFO] Move 1 Error: " + error);
		*/
		drive.startFollowPath(Path.leftCloseSwitch);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		Timer.delay(0.25);
		gripper.lowerGripper();
		gripper.openGripper();
		Timer.delay(0.5);
		drive.turnToHeading(-235, true);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		gripper.liftGripper();
		drive.drivePosition(28, 28);
		double error = drive.getError();
		System.out.println("[INFO] Move 2 Error: " + error);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		
		drive.turnToHeading(-207.5, false);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.turnToHeading(-173, true);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		gripper.lowerGripper();
		Timer.delay(0.5);
		gripper.closeGripper();
		Timer.delay(0.5);
		gripper.liftGripper();
		lifter.liftToHeight(RobotMap.SWITCH_FENCE);
		while(!(lifter.atPosition(RobotMap.SWITCH_FENCE) || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.drivePosition(-12, -12);
		Timer.delay(0.5);
		gripper.lowerGripper();
		gripper.openGripper();
		Timer.delay(0.25);
		drive.drivePosition(32, 32);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		gripper.liftGripper();
		
	}
	
	public static void leftCloseSwitchMove(DriveTrain drive, Gripper gripper, Lifter lifter, boolean twoCube)
	{
		lifter.liftToHeight(RobotMap.SWITCH_FENCE);
		/*
		drive.drivePosition(-108, -108);
		double error = drive.getError();
		System.out.println("[INFO] Move 1 Error: " + error);
		*/
		drive.startFollowPath(Path.leftCloseSwitch);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		Timer.delay(0.25);
		gripper.lowerGripper();
		gripper.openGripper();
		Timer.delay(0.5);
		if(twoCube)
		{
			drive.turnToHeading(-235, true);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
			gripper.liftGripper();
			drive.drivePosition(28, 28);
			double error = drive.getError();
			System.out.println("[INFO] Move 2 Error: " + error);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			
			drive.turnToHeading(-207.5, false);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(-173, true);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.lowerGripper();
			Timer.delay(0.5);
			gripper.closeGripper();
			Timer.delay(0.5);
			gripper.liftGripper();
			lifter.liftToHeight(RobotMap.SWITCH_FENCE);
			while(!(lifter.atPosition(RobotMap.SWITCH_FENCE) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-12, -12);
			Timer.delay(0.5);
			gripper.lowerGripper();
			gripper.openGripper();
			Timer.delay(0.25);
			drive.drivePosition(32, 32);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		}
		gripper.liftGripper();
	}
	
	public static void rightCloseSwitchMove(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		lifter.liftToHeight(RobotMap.SWITCH_FENCE);
		/*
		drive.drivePosition(-114, -114);
		double error = drive.getError();
		System.out.println("[INFO] Move 1 Error: " + error);
		*/
		drive.startFollowPath(Path.rightCloseSwitch);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		Timer.delay(0.5);
		gripper.lowerGripper();
		gripper.openGripper();
		Timer.delay(0.5);
		drive.turnToHeading(235, false);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		gripper.liftGripper();
		drive.drivePosition(28, 28);
		double error = drive.getError();
		System.out.println("[INFO] Move 2 Error: " + error);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		
		drive.turnToHeading(207.5, true);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.turnToHeading(173, false);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		gripper.lowerGripper();
		Timer.delay(0.5);
		gripper.closeGripper();
		Timer.delay(0.5);
		gripper.liftGripper();
		lifter.liftToHeight(RobotMap.SWITCH_FENCE);
		while(!(lifter.atPosition(RobotMap.SWITCH_FENCE) || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.drivePosition(-12, -12);
		Timer.delay(0.5);
		gripper.lowerGripper();
		gripper.openGripper();
		Timer.delay(0.25);
		drive.drivePosition(32, 32);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		gripper.liftGripper();
	}
	
	public static void rightCloseSwitchMove(DriveTrain drive, Gripper gripper, Lifter lifter, boolean twoCube)
	{
		lifter.liftToHeight(RobotMap.SWITCH_FENCE);
		/*
		drive.drivePosition(-114, -114);
		double error = drive.getError();
		System.out.println("[INFO] Move 1 Error: " + error);
		*/
		drive.startFollowPath(Path.rightCloseSwitch);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		Timer.delay(0.5);
		gripper.lowerGripper();
		gripper.openGripper();
		Timer.delay(0.5);
		if(twoCube)
		{
			drive.turnToHeading(235, false);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
			gripper.liftGripper();
			drive.drivePosition(28, 28);
			double error = drive.getError();
			System.out.println("[INFO] Move 2 Error: " + error);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			
			drive.turnToHeading(207.5, true);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(173, false);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.lowerGripper();
			Timer.delay(0.5);
			gripper.closeGripper();
			Timer.delay(0.5);
			gripper.liftGripper();
			lifter.liftToHeight(RobotMap.SWITCH_FENCE);
			while(!(lifter.atPosition(RobotMap.SWITCH_FENCE) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-12, -12);
			Timer.delay(0.5);
			gripper.lowerGripper();
			gripper.openGripper();
			Timer.delay(0.25);
			drive.drivePosition(32, 32);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		}
		gripper.liftGripper();
	}
	
	public static void rightFarSwitchMove(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		drive.startFollowPath(Path.rightFarSwitch);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.turnToHeading(-180, false);
		Timer.delay(1.5);
		drive.driveVBus(-0.25,  -0.25);
		Timer.delay(0.5);
		drive.driveVBus(0, 0);
		lifter.liftToHeight(RobotMap.SCALE_DOWN);
		while(!(lifter.atPosition(RobotMap.SCALE_DOWN) || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		Timer.delay(0.5);
		gripper.openGripper();
	}
	
	public static void leftFarSwitchMove(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		drive.startFollowPath(Path.leftFarSwitch);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.turnToHeading(180, true);
		Timer.delay(1.5);
		drive.driveVBus(-0.25,  -0.25);
		Timer.delay(0.5);
		drive.driveVBus(0, 0);
		lifter.liftToHeight(RobotMap.SCALE_DOWN);
		while(!(lifter.atPosition(RobotMap.SCALE_DOWN) || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		Timer.delay(0.5);
		gripper.openGripper();
	}
	
	public static void rightFarScale(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		if(PathGenerator.isPathGenerated(Path.rightFarScale))
		{
			drive.startFollowPath(Path.rightFarScale);
			while(!drive.doneWithCurrentMove() || autoDone());
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(0, 1, true);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(lifter.atPosition(RobotMap.MAX_SAFE_HEIGHT) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-26, -26);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			Timer.delay(0.5);
			drive.drivePosition(32, 32);
			Timer.delay(0.5);
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
		} 
		else
		{
			drive.drivePosition(-200, -200);
			double error = drive.getError();
			System.out.println("[INFO] Move 1 Error: " + error);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(-90);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-158, -158);
			while(!drive.doneWithCurrentMove() || autoDone());
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(0, true);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(lifter.atPosition(RobotMap.MAX_SAFE_HEIGHT) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-31, -31);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			Timer.delay(0.05);
			drive.drivePosition(31, 31);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		}
	}
	public static void leftFarScale(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		if(PathGenerator.isPathGenerated(Path.leftFarScale))
		{
			drive.startFollowPath(Path.leftFarScale);
			while(!drive.doneWithCurrentMove() || autoDone());
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(0, 1, false);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(lifter.atPosition(RobotMap.MAX_SAFE_HEIGHT) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-26, -26);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			Timer.delay(0.5);
			drive.drivePosition(32, 32);
			Timer.delay(0.5);
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
		} 
		else
		{
			drive.drivePosition(-200, -200);
			double error = drive.getError();
			System.out.println("[INFO] Move 1 Error: " + error);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(90, true);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-158, -158);
			while(!drive.doneWithCurrentMove() || autoDone());
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(0, false);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(lifter.atPosition(RobotMap.MAX_SAFE_HEIGHT) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-31, -31);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			Timer.delay(0.05);
			drive.drivePosition(31, 31);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		}
	}
	
	public static void leftCloseScale(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		if(PathGenerator.isPathGenerated(Path.leftCloseScale))
		{
			drive.startFollowPath(Path.leftCloseScale);
			lifter.liftToHeight(RobotMap.SWITCH_FENCE);
			drive.startFollowPath(PathGenerator.Path.leftCloseScale);
			while(!((drive.getEncoderPos() < -115) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			gripper.openGripper();
			Timer.delay(0.5);
			drive.drivePosition(35, 35);
			Timer.delay(0.25);
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(-185, false);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.liftGripper();
			drive.drivePosition(-44, -44);
			Timer.delay(1.75);
			drive.driveVBus(0, 0);
			gripper.lowerGripper();
			Timer.delay(0.5);
			gripper.closeGripper();
			Timer.delay(0.5);
			drive.drivePosition(23, 23);
			gripper.liftGripper();
			gripper.liftGripper();
		} 
		else
		{
			drive.drivePosition(-240, -240);
			double error = drive.getError();
			System.out.println("[INF0] Move 1 error: " + error);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(-45);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(lifter.atPosition(RobotMap.MAX_SAFE_HEIGHT) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-28, -28);
			while(!(drive.doneAtPosition(-24) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			Timer.delay(0.5);
			drive.drivePosition(28, 28);
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(-180);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.liftGripper();
		}
	}
	
	public static void leftCloseScaleAlt(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		if(PathGenerator.isPathGenerated(Path.leftCloseScaleAlt))
		{
			drive.startFollowPath(Path.leftCloseScaleAlt);
			lifter.liftToHeight(RobotMap.SWITCH_FENCE);
			drive.startFollowPath(PathGenerator.Path.leftCloseScale);
			while(!((drive.getEncoderPos() < -115) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			Timer.delay(0.5);
			drive.drivePosition(35, 35);
			Timer.delay(0.25);
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
		} 
		else 
		{
			AutoTool.abortAuto(drive);
		}
	}
	
	public static void rightCloseScale(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		if(PathGenerator.isPathGenerated(Path.rightCloseScale))
		{
			drive.startFollowPath(Path.rightCloseScale);
			lifter.liftToHeight(RobotMap.SWITCH_FENCE);
			drive.startFollowPath(PathGenerator.Path.rightCloseScale);
			while(!((drive.getEncoderPos() < -115) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			Timer.delay(0.5);
			drive.drivePosition(36, 36);
			Timer.delay(0.25);
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(170, true);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.liftGripper();
			drive.drivePosition(-44, -44);
			Timer.delay(1.75);
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.lowerGripper();
			Timer.delay(0.5);
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.closeGripper();
			Timer.delay(0.5);
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(23, 23);
			gripper.liftGripper();
			drive.turnToHeading(50, 1, true);
			gripper.liftGripper();
			lifter.liftToHeight(RobotMap.SWITCH_FENCE);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			drive.drivePosition(-50, -50);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(0, false);
			Timer.delay(0.25);
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			
			
		} 
		else
		{
			drive.drivePosition(-240, -240);
			double error = drive.getError();
			System.out.println("[INF0] Move 1 error: " + error);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(45, true);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(lifter.atPosition(RobotMap.MAX_SAFE_HEIGHT) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.drivePosition(-28, -28);
			while(!(drive.doneAtPosition(-24) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			Timer.delay(0.5);
			drive.drivePosition(28, 28);
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			drive.turnToHeading(180, true);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.liftGripper();
		}
	}
	
	public static void rightCloseScaleAlt(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		if(PathGenerator.isPathGenerated(Path.rightCloseScaleAlt))
		{
			drive.startFollowPath(Path.rightCloseScaleAlt);
			lifter.liftToHeight(RobotMap.SWITCH_FENCE);
			while(!((drive.getEncoderPos() < -115) || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
			while(!(drive.doneWithCurrentMove() || autoDone()));
			if(autoDone())
			{
				AutoTool.abortAuto(drive);
				return;
			}
			gripper.openGripper();
			Timer.delay(0.5);
			drive.drivePosition(36, 36);
			Timer.delay(0.25);
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		}
		else
		{
			AutoTool.abortAuto(drive);
		}
	}
	
	public static void autoLine(DriveTrain drive)
	{
		Timer.delay(5);
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.drivePosition(-126, -126);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.driveVBus(0, 0);
	}
	
	public static void middleRight(DriveTrain drive, Gripper gripper, Lifter lifter)
	{
		
		lifter.liftToHeight(RobotMap.SWITCH_FENCE);
		drive.drivePosition(-105, -105);
		Timer.delay(2);
		gripper.lowerGripper();
		Timer.delay(0.0e2);
		drive.driveVBus(0, 0);
		gripper.openGripper();
		Timer.delay(1);
		gripper.liftGripper();
		drive.turnToHeading(-45, 1, true);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.drivePosition(79, 79);
		lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.turnToHeading(0, 1, true);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.drivePosition(-18, -18);
		Timer.delay(1);
		gripper.lowerGripper();
		Timer.delay(1);
		gripper.closeGripper();
		Timer.delay(0.5);
		drive.drivePosition(12, 12);
		gripper.liftGripper();
	}
	
	public static void middleLeft(DriveTrain drive, Gripper gripper, Lifter lifter) {
		lifter.liftToHeight(RobotMap.SWITCH_FENCE);
		drive.drivePosition(-2, -2);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.turnToHeading(62, 1, true);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.drivePosition(-95, -95);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.turnToHeading(0, 1, false);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.drivePosition(-15, -15);
		Timer.delay(1);
		gripper.lowerGripper();
		gripper.openGripper();
		Timer.delay(1);
		drive.turnToHeading(45, 1, false);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.drivePosition(82, 82);
		lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		gripper.liftGripper();
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.turnToHeading(0, 1, false);
		while(!(drive.doneWithCurrentMove() || autoDone()));
		if(autoDone())
		{
			AutoTool.abortAuto(drive);
			return;
		}
		drive.drivePosition(-18, -18);
		Timer.delay(1);
		gripper.lowerGripper();
		Timer.delay(1);
		gripper.closeGripper();
		Timer.delay(0.5);
		drive.drivePosition(12, 12);
		gripper.liftGripper();
	}
	
	public static boolean autoDone()
	{
		return !(Robot.DS.isAutonomous());
	}
}
