package org.usfirst.frc.team4910.util.path;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team4910.iterator.Iterator;
import org.usfirst.frc.team4910.robot.RobotMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.modifiers.TankModifier;

public class PathGenerator {
	
	public static void main(String args[])
	{
		System.loadLibrary("pathfinderjava");
		generatePaths();
	}
	
	public enum Path
	{
		PIDTest,
		middleLeft,
		middleLeft2,
		middleRight,
		middleRight2,
		leftCloseSwitch,
		rightCloseSwitch,
		leftFarSwitch,
		rightFarSwitch,
		leftCloseScale,
		leftCloseScaleAlt,
		rightCloseScale,
		rightCloseScaleAlt,
		leftFarScale,
		rightFarScale,
	}
	
	private static Waypoint[] middleLeftPath = {
			new Waypoint(0, 0, Pathfinder.d2r(61)),
			new Waypoint(38, 65, Pathfinder.d2r(61)),
	};
	
	private static Waypoint[] middleRightPath = {
			new Waypoint(0, 0, 0),
			new Waypoint(105, 0, 0)
	};
	
	private static Waypoint[] PIDTestPath = {
			new Waypoint(0, 0, Pathfinder.d2r(-45)),
			new Waypoint(-52, -48, Pathfinder.d2r(-180))
	};
	
	private static Waypoint[] leftCloseSwitchPath = {
			new Waypoint(0, 0, 0),
			new Waypoint(98, 0, 0),
			new Waypoint(144, -38, Pathfinder.d2r(-90))
		};
	
	private static Waypoint[] rightCloseSwitchPath = {
			new Waypoint(0, 0, 0),
			new Waypoint(98, 0, 0),
			new Waypoint(144, 38, Pathfinder.d2r(90))
	};
	
	private static Waypoint[] leftFarSwitchPath = {
			new Waypoint (0, 0, 0),
			new Waypoint (125, 0, 0),
			new Waypoint (238, 136, Pathfinder.d2r(90)),
	};
	
	private static Waypoint[] rightFarSwitchPath = {
			new Waypoint (0, 0, 0),
			new Waypoint (125, 0, 0),
			new Waypoint (238, -136, Pathfinder.d2r(-90)),
	};
	
	private static Waypoint[] leftCloseScalePath = {
			new Waypoint(0, 0, 0),
			new Waypoint(220, 0, 0),
			new Waypoint(273, -36, Pathfinder.d2r(-45))
			
	};
	
	private static Waypoint[] leftCloseScaleAltPath = {
		new Waypoint(0, 0, 0),
		new Waypoint(245, 0, 0),
		new Waypoint(290, -24, Pathfinder.d2r(-90))
	};
	
	private static Waypoint[] rightCloseScalePath = {
			new Waypoint (0, 0, 0),
			new Waypoint(226, 0, 0),
			new Waypoint(280, 24, Pathfinder.d2r(45))
	};
	
	private static Waypoint[] rightCloseScaleAltPath = {
			new Waypoint(0, 0, 0),
			new Waypoint(245, 0, 0),
			new Waypoint(290, 24, Pathfinder.d2r(90))
	};
	
	private static Waypoint[] middleLeft2Path = {
			new Waypoint(0, 0, 180),
			new Waypoint(42, 54, 180)
	};
	
	private static Waypoint[] middleRight2Path = {
			new Waypoint(0, 0, 180),
			new Waypoint(42, -54, 180)
	};
	
	private static Waypoint[] leftFarScalePath = {
			new Waypoint (0, 0, 0),
			new Waypoint (120, 0, 0),
			new Waypoint (226, 176, Pathfinder.d2r(90)),
	};
	
	private static Waypoint[] farScalePath2 = {
			new Waypoint (0, 0, 0),
			new Waypoint (25, 0, 0),
	};
	
	private static Waypoint[] rightFarScalePath = {
			new Waypoint (0, 0, 0),
			new Waypoint (123, 0, 0),
			new Waypoint (226, -176, Pathfinder.d2r(-90))
	};
	
	private static Map<Path, Waypoint[]> pathTable = new HashMap<Path, Waypoint[]>();
	
	public static String rightEnd = "_RightDrive.csv";
	public static String leftEnd  =  "_LeftDrive.csv";
	
	private static Trajectory.Config config = new Trajectory.Config(
			Trajectory.FitMethod.HERMITE_QUINTIC,
			Trajectory.Config.SAMPLES_HIGH,
			Iterator.kPeriod,
			108,
			76,
			RobotMap.maxJerkPath);
	
	static 
	{
		pathTable.put(Path.middleLeft, middleLeftPath);
		pathTable.put(Path.PIDTest, PIDTestPath);
		pathTable.put(Path.leftCloseSwitch, leftCloseSwitchPath);
		pathTable.put(Path.rightCloseSwitch,  rightCloseSwitchPath);
		pathTable.put(Path.leftCloseScale, leftCloseScalePath);
		pathTable.put(Path.rightCloseScale, rightCloseScalePath);
		pathTable.put(Path.middleLeft2, middleLeft2Path);
		pathTable.put(Path.middleRight2, middleRight2Path);
		pathTable.put(Path.leftFarScale, leftFarScalePath);
		pathTable.put(Path.rightFarScale, rightFarScalePath);
		pathTable.put(Path.middleRight, middleRightPath);
		pathTable.put(Path.leftFarSwitch, leftFarSwitchPath);
		pathTable.put(Path.rightFarSwitch, rightFarSwitchPath);
		pathTable.put(Path.leftCloseScaleAlt, leftCloseScaleAltPath);
		pathTable.put(Path.rightCloseScaleAlt, rightCloseScaleAltPath);
	}
	
	public static void generatePaths()
	{
		System.out.println("[INFO] Generating all paths...");
		for(Path path: Path.values())
		{
			generatePath(path);
		}
		System.out.println("[OK] Finished.");
	}
	
	public static void generatePath(Path path)
	{
		Trajectory trajectory;
		File left;
		File right;
		Trajectory leftTrajectory;
		Trajectory rightTrajectory;
		System.out.println("[INFO] Generating " + path + " path...");
		trajectory = Pathfinder.generate(pathTable.get(path), config);
		
		TankModifier mod = new TankModifier(trajectory).modify(RobotMap.weelbase);
		
		leftTrajectory = mod.getLeftTrajectory();
		rightTrajectory = mod.getRightTrajectory();
		
		left = new File(path +  leftEnd);
		right = new File(path + rightEnd);
		
		Pathfinder.writeToCSV(right, rightTrajectory);
		Pathfinder.writeToCSV(left, leftTrajectory);
		System.out.println("[OK] Done.");
	}
	
	public static void generatePath(Path path, Trajectory.Config c)
	{
		Trajectory trajectory;
		File left;
		File right;
		Trajectory leftTrajectory;
		Trajectory rightTrajectory;
		System.out.println("[INFO] Generating " + path + " path...");
		trajectory = Pathfinder.generate(pathTable.get(path), c);
		
		TankModifier mod = new TankModifier(trajectory).modify(RobotMap.weelbase);
		
		leftTrajectory = mod.getLeftTrajectory();
		rightTrajectory = mod.getRightTrajectory();
		
		left = new File(path +  leftEnd);
		right = new File(path + rightEnd);
		
		Pathfinder.writeToCSV(right, rightTrajectory);
		Pathfinder.writeToCSV(left, leftTrajectory);
		System.out.println("[OK] Done.");
	}
	
	public static boolean isPathGenerated(Path path)
	{
		System.out.println("[INFO] Checking to see if " + path + "has been generated...");
		boolean file1 = new File(path + leftEnd).isFile();
		boolean file2 = new File(path + rightEnd).isFile();
		if(file1 && file2)
		{
			System.out.println("[OK] " + path + " has been generated!");
			return true;
		}
		else
		{
			System.out.println("[WARNING] " + path + " has not been genereated.");
			return false;
		}
	}
	
	public static boolean arePathsGenerated()
	{
		System.out.println("[INFO] Checking Paths...");
		boolean temp = true;
		for(Path path: Path.values())
		{
			if(!isPathGenerated(path))
			{
				temp = false;
			}
		}
		return temp;
	}
	
	public static Trajectory invenrtPath(Trajectory path)
	{
		Segment[] inverted_segments = new Segment[path.segments.length];
		int index = 0;
		for(Segment seg: path.segments )
		{
			seg.position = -seg.position;
			seg.velocity = -seg.velocity;
			inverted_segments[index] = seg;
			index++;
		}
		
		return new Trajectory(inverted_segments);
	}
}
