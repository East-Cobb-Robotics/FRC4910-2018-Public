package org.usfirst.frc.team4910.robot;

import org.usfirst.frc.team4910.auto.AutoCommand;
import org.usfirst.frc.team4910.iterator.Iterator;
import org.usfirst.frc.team4910.subsystems.DriveTrain;
import org.usfirst.frc.team4910.subsystems.DriveTrain.DriveState;
import org.usfirst.frc.team4910.subsystems.Gripper;
import org.usfirst.frc.team4910.subsystems.Gripper.GripState;
import org.usfirst.frc.team4910.subsystems.Lifter;
import org.usfirst.frc.team4910.subsystems.Lifter.LiftState;
import org.usfirst.frc.team4910.util.LIDAR;
import org.usfirst.frc.team4910.util.LidarLitePWM;
import org.usfirst.frc.team4910.util.PressureGauge;
import org.usfirst.frc.team4910.util.path.PathGenerator;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot 
{
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private static final String kDefaultTest = "System Test";
	private static final String kPIDTest = "PIDTest";
	private static final String kPIDTurn = "PIDTurn";
	private static final String kPIDPathTest = "PIDPathTest";
	private static final String kGeneratePaths = "GeneratePaths";
	private static final String kLIDARTest = "LIDAR Test";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private SendableChooser<String> testChooser = new SendableChooser<>();
	private SendableChooser<String> positionChooser = new SendableChooser<>();
	private static final String kPosChoose = "Choose Robot Position";
	private static final String kLeftPosition = "Left Position";
	private static final String kMidPosition = "Middle Position";
	private static final String kRightPosition = "Right Position";
	private SendableChooser<String> objectiveChooser = new SendableChooser<>();
	private static final String kObjChoose = "Choose Objective(s)";
	private static final String kAutoLine = "Auto Line ONLY";
	private static final String kScSw = "Scale to Switch";
	private static final String kSwSc = "Switch to Scale";
	private static final String kSwitch = "Switch ONLY";
	private static final String kScale = "Scale ONLY";
	private SendableChooser<String> pathChooser = new SendableChooser<>();
	private static final String kPathChoose = "Choose Robot Path";
	private static final String kDirectPath = "Direct Path";
	private static final String kAroundPath = "Around Path";
	public static DriverStation DS;
	private static Joystick lJoy1, rJoy1, rJoy2;
	
	private static PressureGauge gauge;
	
	private static PowerDistributionPanel PDP;
	
	private double[] robotValues;
	
	private boolean dualStick;
	private boolean isHigh;

	private static Iterator enabledIterator, disabledIterator;
	
	private DriveTrain drive;
	private Compressor comp;
	private Gripper gripper;
	private Lifter lifter;
	
	//private LidarLitePWM lidar;
	private DigitalInput lidarPWM;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		testChooser.addDefault("System Test", kDefaultTest);
		testChooser.addObject("PIDTest", kPIDTest);
		testChooser.addObject("PIDTurn", kPIDTurn);
		testChooser.addObject("PIDPathTest", kPIDPathTest);
		testChooser.addObject("GeneratePaths", kGeneratePaths);
		testChooser.addObject("LIDAR Test", kLIDARTest);
		SmartDashboard.putData("Testng Choices", testChooser);
		
		positionChooser.addDefault("Choose Robot Position", kPosChoose);
		positionChooser.addObject("Left Position", kLeftPosition);
		positionChooser.addObject("Middle Position", kMidPosition);
		positionChooser.addObject("Right Position", kRightPosition);
		SmartDashboard.putData("Robot Position", positionChooser);
		
		objectiveChooser.addDefault("Choose Objective(s)", kObjChoose);
		objectiveChooser.addObject("Auto Line ONLY", kAutoLine);
		objectiveChooser.addObject("Scale to Switch" , kScSw);
		objectiveChooser.addObject("Switch to Scale", kSwSc);
		objectiveChooser.addObject("Switch ONLY", kSwitch);
		objectiveChooser.addObject("Scale ONLY", kScale);
		SmartDashboard.putData("Auto Objective(s)", objectiveChooser);
		
		PDP = new PowerDistributionPanel();
		
		SmartDashboard.putData("PDP", PDP);
		
		DS = DriverStation.getInstance();
		lJoy1 = new Joystick(RobotMap.leftJoy1Port);
		rJoy1 = new Joystick(RobotMap.rightJoy1Port);
		
		//lJoy2 = new Joystick(RobotMap.leftJoy2Port);

		rJoy2 = new Joystick(RobotMap.rightJoy2Port);

		drive = DriveTrain.getInstance();

		drive.setBrakeMode(true);
		
		gripper = Gripper.getInstance();
		gripper.closeGripper();
		lifter = Lifter.getInstance();
		
		lidarPWM = new DigitalInput(4);
		//lidar = new LidarLitePWM(lidarPWM);
		comp = new Compressor();
		
		enabledIterator = new Iterator();
		
		enabledIterator.register(gripper.getIterate());
		enabledIterator.register(lifter.getIterate());
		enabledIterator.register(drive.getIterate());
		
		dualStick = false;
		isHigh = false;
		
		gauge = new PressureGauge(0);
		SmartDashboard.putNumber("Air Pressure", gauge.getPressure());
		
		robotValues = new double[]{comp.getCompressorCurrent(), RobotController.getBatteryVoltage()};
		
		SmartDashboard.putBoolean("Low Gear", drive.getGear());
		SmartDashboard.putNumberArray("Robot Values", robotValues);

		SmartDashboard.putBoolean("Gripper Open", gripper.getState() == GripState.Open);
		SmartDashboard.putBoolean("Gripper Up", gripper.getHeight());
		SmartDashboard.putBoolean("Climber Gear", lifter.getState() == LiftState.Climbing);
		SmartDashboard.putBoolean("Two People: GREEN", dualStick);
		SmartDashboard.putData("NavX Heading", drive.getNavXObj());
		
		SmartDashboard.putNumber("Gyro kP", RobotMap.gyroKp);
		SmartDashboard.putNumber("Gyro kI", RobotMap.gyroKi);
		SmartDashboard.putNumber("Gyro kD", RobotMap.gyroKd);
		SmartDashboard.putNumber("Gyro kF", RobotMap.gyroKf);
		SmartDashboard.putNumber("GyroIZone", RobotMap.gyroIzone);
		SmartDashboard.putNumber("Error Time", RobotMap.gyroTolernaceZoneTime);
		SmartDashboard.putNumber("Turn", 89);
	}
	
	public void robotPeriodic()
	{
			robotValues[0] = comp.getCompressorCurrent();
			robotValues[1] = RobotController.getBatteryVoltage();
			
			SmartDashboard.putNumberArray("Robot Values", robotValues);
			
			SmartDashboard.putString("Drive Train State", drive.getState().toString());
			
			SmartDashboard.putString("Lifter State", lifter.getState().toString());
			SmartDashboard.putBoolean("Climber Gear", lifter.getGear());
			
			SmartDashboard.putString("Gripper State", gripper.getState().toString());
			SmartDashboard.putBoolean("Gripper Up", gripper.getHeight());
			SmartDashboard.putBoolean("Gripper Open", gripper.getState() == GripState.Open);
			SmartDashboard.putBoolean("Two People: GREEN", dualStick);
			SmartDashboard.putNumber("Air Pressure", gauge.getPressure());
	}
	
	public void disabledInit()
	{
		enabledIterator.stop();
		//lidar.stop();
	}
	
	public void disabledPeriodic(){
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		
		String autoKey = "";
		
		System.out.println("Auto selected: " + m_autoSelected);
		enabledIterator.start();
		lifter.reset();
		drive.setRampMode(DriveTrain.RampMode.None);
		drive.resetNavX();
		
		switch(positionChooser.getSelected()){
		case kLeftPosition:
			autoKey = "0";
			System.out.println("LEFT Position");
			break;
		case kMidPosition:
			autoKey = "1";
			System.out.println("MIDDLE Position");
			break;
		case kRightPosition:
			autoKey = "2";
			System.out.println("RIGHT Position");
			break;
		default:
			autoKey = "3";
			System.out.println("UNKNOWN Position");
			break;
		}
		
		switch(DS.getGameSpecificMessage()){
		case "LLL":
			autoKey = autoKey + "0";
			System.out.println("LLL");
			break;
		case "LRL":
			autoKey = autoKey + "1";
			System.out.println("LRL");
			break;
		case "RLR":
			autoKey = autoKey + "2";
			System.out.println("RLR");
			break;
		case "RRR":
			autoKey = autoKey + "3";
			System.out.println("RRR");
			break;
		default:
			autoKey = autoKey + "4";
			System.out.println("UNKNOWN Switch/Scale Position");
			break;
		}
		
		switch(objectiveChooser.getSelected()){
		case kAutoLine:
			autoKey = autoKey + "00";
			System.out.println("AUTO LINE");
			break;
		case kScSw:
			autoKey = autoKey + "21";
			System.out.println("Scale Switch");
			break;
		case kSwSc:
			autoKey = autoKey + "12";
			System.out.println("Switch Scale");
			break;
		case kSwitch:
			autoKey = autoKey + "11";
			System.out.println("Switch Only");
			break;
		case kScale:
			autoKey = autoKey + "22";
			System.out.println("Scale Only");
			break;
		default:
			autoKey = autoKey + "33";
			System.out.println("UNKNOWN Objective");
			break;
		}
		
		String autoDecipher = "I am error. Great Scott!", autoPos, autoObj1, autoObj2, autoSw, autoSc;
		try{
		switch(autoKey.charAt(0)){
		case '0':
			autoPos = "LEFT position";
			switch(autoKey.charAt(1)){ 
			case '0':
				autoSw = "on the LEFT";
				autoSc = "on the LEFT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					AutoCommand.autoLine(drive);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					AutoCommand.leftCloseScale(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					AutoCommand.leftCloseSwitchMove(drive, gripper, lifter, false);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.leftCloseSwitchMove(drive, gripper, lifter, false);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.leftCloseScale(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '1':
				autoSw = "on the LEFT";
				autoSc = "on the RIGHT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					AutoCommand.rightFarScale(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					AutoCommand.leftCloseSwitchMove(drive, gripper, lifter, true);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.leftCloseSwitchMove(drive, gripper, lifter, true);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.rightFarScale(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '2':
				autoSw = "on the RIGHT";
				autoSc = "on the LEFT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					AutoCommand.leftCloseScale(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					AutoCommand.autoLine(drive);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.autoLine(drive);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.leftCloseScale(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '3':
				autoSw = "on the RIGHT";
				autoSc = "on the RIGHT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					AutoCommand.rightFarScale(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					AutoCommand.autoLine(drive);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.autoLine(drive);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.rightFarScale(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '4':
				autoSw = "in an UNKNOWN position";
				autoSc = "in an UNKNOWN position";
				autoDecipher = "We know we're starting in " + autoPos + ", but don't know what positions the SCALE and SWITCH are starting in! Defaulting to AUTO LINE.";
				AutoCommand.autoLine(drive);
				break;
			}
			break;
		case '1':
			autoPos = "MIDDLE position";
			switch(autoKey.charAt(1)){ 
			case '0':
				autoSw = "on the LEFT";
				autoSc = "on the LEFT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.middleLeft(drive, gripper, lifter);
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.middleLeft(drive, gripper, lifter);
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.middleLeft(drive, gripper, lifter);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					AutoCommand.middleLeft(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '1':
				autoSw = "on the LEFT";
				autoSc = "on the RIGHT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.middleLeft(drive, gripper, lifter);
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.middleLeft(drive, gripper, lifter);
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.middleLeft(drive, gripper, lifter);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.middleLeft(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '2':
				autoSw = "on the RIGHT";
				autoSc = "on the LEFT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.middleRight(drive, gripper, lifter);
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.middleRight(drive, gripper, lifter);
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.middleRight(drive, gripper, lifter);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.middleRight(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '3':
				autoSw = "on the RIGHT";
				autoSc = "on the RIGHT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.middleRight(drive, gripper, lifter);
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.middleRight(drive, gripper, lifter);
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.middleRight(drive, gripper, lifter);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.middleRight(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '4':
				autoSw = "in an UNKNOWN position";
				autoSc = "in an UNKNOWN position";
				autoDecipher = "We know we're starting in " + autoPos + ", but don't know what positions the SCALE and SWITCH are starting in! Defaulting to AUTO LINE.";
				AutoCommand.autoLine(drive);
				break;
			}
			break;
		case '2':
			autoPos = "RIGHT position";
			switch(autoKey.charAt(1)){ 
			case '0':
				autoSw = "on the LEFT";
				autoSc = "on the LEFT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					AutoCommand.leftFarScale(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					AutoCommand.autoLine(drive);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.autoLine(drive);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.leftFarScale(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '1':
				autoSw = "on the LEFT";
				autoSc = "on the RIGHT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					AutoCommand.rightCloseScale(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					AutoCommand.autoLine(drive);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.autoLine(drive);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.rightCloseScale(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '2':
				autoSw = "on the RIGHT";
				autoSc = "on the LEFT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					AutoCommand.autoLine(drive);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					AutoCommand.leftFarScale(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					AutoCommand.rightCloseSwitchMove(drive, gripper, lifter, true);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.rightCloseSwitchMove(drive, gripper, lifter, true);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					AutoCommand.leftFarScale(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '3':
				autoSw = "on the RIGHT";
				autoSc = "on the RIGHT";
				switch(autoKey.substring(2)){
				case "00":
					autoObj1 = "AUTO LINE";
					autoObj2 = "STOP";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " and then " + autoObj2;
					AutoCommand.autoLine(drive);
					break;
				case "21":
					autoObj1 = "SCALE";
					autoObj2 = "SWITCH";
					AutoCommand.rightCloseScale(drive, gripper, lifter);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSw;
					break;
				case "12":
					autoObj1 = "SWITCH";
					autoObj2 = "SCALE";
					AutoCommand.rightCloseSwitchMove(drive, gripper, lifter, false);
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSc;
					break;
				case "11":
					autoObj1 = "SWITCH";
					autoObj2 = "SWITCH";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSw + " and then the " + autoObj2 + " " + autoSw;
					AutoCommand.rightCloseSwitchMove(drive, gripper, lifter, false);
					break;
				case "22":
					autoObj1 = "SCALE";
					autoObj2 = "SCALE";
					autoDecipher = "Starting in " + autoPos + ", go to the " + autoObj1 + " " + autoSc + " and then the " + autoObj2 + " " + autoSc;
					AutoCommand.rightCloseScale(drive, gripper, lifter);
					break;
				case "33":
					autoObj1 = "UNKNOWN OBJECTIVE";
					autoObj2 = "UNKNOWN OBJECTIVE";
					autoDecipher = "We started in " + autoPos + ", but we don't we don't have any known objectives. Defaulting to AUTO LINE.";
					AutoCommand.autoLine(drive);
					break;
				}
				break;
			case '4':
				autoSw = "in an UNKNOWN position";
				autoSc = "in an UNKNOWN position";
				autoDecipher = "We know we're starting in " + autoPos + ", but don't know what positions the SCALE and SWITCH are starting in! Defaulting to AUTO LINE.";
				AutoCommand.autoLine(drive);
				break;
			}
			break;
		case '3':
			autoPos = "UNKNOWN position";
			autoDecipher = "We don't know what position we're starting in! Defaulting to AUTO LINE.";
			AutoCommand.autoLine(drive);
			break;
		default:
			System.out.println(autoDecipher + " DEFAULTING TO AUTO LINE");
			AutoCommand.autoLine(drive);
		}
		}
		catch(NullPointerException e){
			e.printStackTrace();
			System.out.println(autoDecipher + " DEFAULTING TO AUTO LINE");
			AutoCommand.autoLine(drive);
		}
		System.out.println(autoDecipher);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		
	}
	
	@Override
	public void teleopInit(){
		comp.setClosedLoopControl(true);
		enabledIterator.start();
		drive.driveVBus(0,0);
		drive.setRampMode(DriveTrain.RampMode.LowLift);
		drive.setHighGear();
		
		gripper.liftGripper();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		double leftDrive = Math.abs(lJoy1.getY()) <= 0.1 ? 0 : lJoy1.getY();
		double rightDrive= Math.abs(rJoy1.getY()) <= 0.1 ? 0 : rJoy1.getY();
		double liftThrottle = 0;
		
		if(!dualStick){
		liftThrottle = rJoy2.getY() < 0 ? -(Math.pow(rJoy2.getY(), 2)) : Math.pow(rJoy2.getY(), 2);	
		}
		else{
			if(rJoy1.getPOV(0) == 0){
				liftThrottle = -.5;
			}
			else if(rJoy1.getPOV(0) == 180){
				liftThrottle = .5;
			}
			else{
				liftThrottle = 0;
			}
		}
		
		drive.driveVBus(leftDrive, rightDrive);
		
		
		if(rJoy1.getRawButton(RobotMap.driveShift)){
			while(rJoy1.getRawButton(RobotMap.driveShift)){};
			drive.shift();	
		}
		
		if(rJoy1.getRawButton(RobotMap.toggleComp)){
			while(rJoy1.getRawButton(RobotMap.toggleComp)){};
			if(comp.enabled()){
				comp.stop();
			}
			else{
				comp.start();
			}
		}
		
		if(rJoy1.getTrigger())
		{
			while(rJoy1.getTrigger());
			if(gripper.getState() == Gripper.GripState.Open)
			{
				gripper.closeGripper();
				Timer.delay(0.5);
				gripper.liftGripper();
			} 
			else
			{
				gripper.lowerGripper();
				Timer.delay(0.05);
				gripper.openGripper();
			}	
		}
		
		if(rJoy1.getRawButton(5))
		{
			while(rJoy1.getRawButton(5));
			gripper.openGripper();
			gripper.liftGripper();
		}
		
		if(rJoy1.getRawButton(RobotMap.toggleGrip)){
			while(rJoy1.getRawButton(RobotMap.toggleGrip));
			System.out.println("Calling toggleGrip @" + Timer.getFPGATimestamp());
			gripper.toggleGrip();
		}
		
		if(rJoy1.getRawButton(RobotMap.toggleHeight)){
			while(rJoy1.getRawButton(RobotMap.toggleHeight));
			System.out.println("Calling toggleHeight @" + Timer.getFPGATimestamp());
			gripper.toggleHeight();
		}
		
		if(!dualStick){
			if(rJoy2.getRawButton(RobotMap.toggleClimb)){
			while(rJoy2.getRawButton(RobotMap.toggleClimb));
			lifter.climbToggle();
			} 
		}
		else{
			if(rJoy1.getRawButton(RobotMap.dualToggleClimb)){
				while(rJoy1.getRawButton(RobotMap.dualToggleClimb));
				lifter.climbToggle();
			}
		}
		
		if(lJoy1.getRawButton(RobotMap.dualStickToggle)){
			while(lJoy1.getRawButton(RobotMap.dualStickToggle));
			dualStick = !dualStick;
		}
		
		if(rJoy2.getTrigger()){
			lifter.liftVBus(liftThrottle);
		}
		else if(lifter.getState() != LiftState.LiftHeight){
			lifter.lockLifter();
		}
		
		if(rJoy2.getRawButton(RobotMap.lockOverride))
		{
			lifter.liftVBus(0);
		}
		
		if(rJoy2.getRawButton(4))
		{
			while(rJoy2.getRawButton(4));
			lifter.reset();
		}
		else if(rJoy2.getRawButton(7))
		{
			while(rJoy2.getRawButton(7));
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		}
		else if(rJoy2.getRawButton(8))
		{
			while(rJoy2.getRawButton(8));
			lifter.liftToHeight(RobotMap.SCALE_DOWN);
		}
		else if(rJoy2.getRawButton(9))
		{
			while(rJoy2.getRawButton(9));
			lifter.liftToHeight(RobotMap.CUBE_STACK);
		}
		else if(rJoy2.getRawButton(10))
		{
			while(rJoy2.getRawButton(10));
			lifter.liftToHeight(RobotMap.MAX_SAFE_HEIGHT);
		}
		else if(rJoy2.getRawButton(11))
		{
			while(rJoy2.getRawButton(11));
			lifter.liftToHeight(RobotMap.SWITCH_FENCE);
		}
		else if(lJoy1.getTrigger())
		{
			while(rJoy1.getTrigger());
			lifter.liftToHeight(RobotMap.LIFTER_DOWN);
		}
	}
	
	/**
	 * This funtion is called at the start of test mode.
	 */
	@Override
	public void testInit()
	{
		drive.setRampMode(DriveTrain.RampMode.None);
		enabledIterator.start();
		if(testChooser.getSelected() == kGeneratePaths)
		{
			PathGenerator.generatePaths();
		}
		//lidar.start(100);
	}
	
	//Temporary vars for Gyro PIDTuning
	private double gyroKp;
	private double gyroKi;
	private double gyroKd;
	private double gyroKf;
	private double gyroIZone;
	private double gyroTime;
	private double turn;

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
			int multiplier = 16;
		if(testChooser.getSelected() == kPIDTest)
		{
			if(lJoy1.getRawButton(RobotMap.PIDBack))
			{
				while(lJoy1.getRawButton(RobotMap.PIDBack));
				drive.drivePosition(12*multiplier, 12*multiplier);
				System.out.println("NavX Heading: " + drive.getNavXHeading());
			}
			
			if(lJoy1.getRawButton(RobotMap.PIDForoward))
			{
				drive.resetNavX();
				while(lJoy1.getRawButton(RobotMap.PIDForoward));
				drive.drivePosition(-12*multiplier, -12*multiplier);
			}
			if(lJoy1.getRawButton(5))
			{
				while(lJoy1.getRawButton(5));
				System.out.println("Reseting encoders");
				drive.resetEncoders();
			}
			
			
		}

		else if(testChooser.getSelected() == kPIDTurn)
		{
			this.gyroKp = SmartDashboard.getNumber("Gyro kP", RobotMap.gyroKp);
			this.gyroKi = SmartDashboard.getNumber("Gyro kI",  RobotMap.gyroKi);
			this.gyroKd = SmartDashboard.getNumber("Gyro kD", RobotMap.gyroKd);
			this.gyroKf = SmartDashboard.getNumber("Gyro kF", RobotMap.gyroKf);
			this.gyroIZone = SmartDashboard.getNumber("GyroIZone", RobotMap.gyroIzone);
			this.gyroTime = SmartDashboard.getNumber("Error Time", RobotMap.gyroTolernaceZoneTime);
			this.turn = SmartDashboard.getNumber("Turn", 89);
			drive.setGyroPID(gyroKp, gyroKi, gyroKd, gyroKf, gyroIZone);
			if(lJoy1.getRawButton(RobotMap.PIDBack))
			{
				
				
				while(lJoy1.getRawButton(RobotMap.PIDBack));
				drive.resetNavX();
				drive.turnToHeading(-turn);
				
			}
			if(lJoy1.getRawButton(RobotMap.PIDForoward))
			{
				while(lJoy1.getRawButton(RobotMap.PIDForoward));
				drive.turnToHeading(turn);
			}
			SmartDashboard.putNumber("NavX Heading", drive.getNavXHeading());
		}
		else if(testChooser.getSelected() == kPIDPathTest)
		{
			if(lJoy1.getRawButton(RobotMap.PIDForoward))
			{
				while(lJoy1.getRawButton(RobotMap.PIDForoward));
				drive.resetEncoders();
				drive.startFollowPath(PathGenerator.Path.PIDTest);
			}
			if(lJoy1.getTrigger())
			{
				while(lJoy1.getTrigger());
				drive.turnToHeading(-45, false);
			}
		}
		else if(testChooser.getSelected() == kLIDARTest) {
			//System.out.printf("LIDAR Dist: %.2f%n", lidar.getDistance());
		}
		
		}
	}		
