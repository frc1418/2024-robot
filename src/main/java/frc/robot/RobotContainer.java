// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.commands.AlignByAprilTag;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LevelChargingStationCommand;
import frc.robot.commands.autonomous.ChargeCommand;
import frc.robot.commands.autonomous.LeftUpperConeAutonomous;
import frc.robot.commands.autonomous.MiddleUpperConeAutonomous;
import frc.robot.commands.autonomous.MiddleAutonomousMiddleCone;
import frc.robot.commands.autonomous.MiddleAutonomousNoCone;
import frc.robot.commands.autonomous.RightUpperConeAutonomous;
import frc.robot.commands.autonomous.UpperConeAutonomous;
import frc.robot.common.Odometry;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WheelSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    private final RobotBase robot;

    private CANSparkMax backRightAngleMotor = new CANSparkMax(DrivetrainConstants.BACK_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backRightSpeedMotor = new CANSparkMax(DrivetrainConstants.BACK_RIGHT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder backRightEncoder = new AnalogEncoder(DrivetrainConstants.BACK_RIGHT_ENCODER);

    private CANSparkMax frontRightAngleMotor = new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontRightSpeedMotor = new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder frontRightEncoder = new AnalogEncoder(DrivetrainConstants.FRONT_RIGHT_ENCODER);

    private CANSparkMax backLeftAngleMotor = new CANSparkMax(DrivetrainConstants.BACK_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backLeftSpeedMotor = new CANSparkMax(DrivetrainConstants.BACK_LEFT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder backLeftEncoder = new AnalogEncoder(DrivetrainConstants.BACK_LEFT_ENCODER);

    private CANSparkMax frontLeftAngleMotor = new CANSparkMax(DrivetrainConstants.FRONT_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontLeftSpeedMotor = new CANSparkMax(DrivetrainConstants.FRONT_LEFT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder frontLeftEncoder = new AnalogEncoder(DrivetrainConstants.FRONT_LEFT_ENCODER);

    private WheelSubsystem backRightWheel = new WheelSubsystem (
        backRightAngleMotor, backRightSpeedMotor, backRightEncoder,
        DrivetrainConstants.BACK_RIGHT_LOC);
    public WheelSubsystem backLeftWheel = new WheelSubsystem (
      backLeftAngleMotor, backLeftSpeedMotor, backLeftEncoder,
      DrivetrainConstants.BACK_LEFT_LOC);
    private WheelSubsystem frontRightWheel = new WheelSubsystem (
      frontRightAngleMotor, frontRightSpeedMotor, frontRightEncoder,
      DrivetrainConstants.FRONT_RIGHT_LOC);
    private WheelSubsystem frontLeftWheel = new WheelSubsystem (
      frontLeftAngleMotor, frontLeftSpeedMotor, frontLeftEncoder,
      DrivetrainConstants.FRONT_LEFT_LOC);
    

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/Odometry");

    private final NetworkTableEntry inclineAngle = table.getEntry("inclineAngle");
    private final NetworkTableEntry inclineDirection = table.getEntry("inclineDirection");

    private final NetworkTableEntry pitch = table.getEntry("pitch");
    private final NetworkTableEntry roll = table.getEntry("roll");

    private SwerveModulePosition[] positions = new SwerveModulePosition[] {
      frontLeftWheel.getSwerveModulePosition(),
      frontRightWheel.getSwerveModulePosition(),
      backLeftWheel.getSwerveModulePosition(),
      backRightWheel.getSwerveModulePosition()
    };

    private SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, gyro.getRotation2d(), positions);

    private LimelightSubsystem limelight = new LimelightSubsystem();
    private Odometry odometry = new Odometry(gyro, driveOdometry, positions, limelight);

    private Spark blinkin = new Spark(LedConstants.BLINKIN_CHANNEL);
    private LedSubsystem ledSubsystem = new LedSubsystem(blinkin);
    
    private SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(
        backRightWheel, backLeftWheel, frontRightWheel, frontLeftWheel,
        DrivetrainConstants.SWERVE_KINEMATICS, odometry);


    private CANSparkMax pivotMotor = new CANSparkMax(ArmConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    private TalonFX telescopeMotor = new TalonFX(ArmConstants.TELESCOPE_MOTOR_ID);
    private PivotSubsystem pivotSubsystem = new PivotSubsystem(pivotMotor);
    private TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem(telescopeMotor, pivotSubsystem);

    private DoubleSolenoid leftSolenoid = new DoubleSolenoid(GrabberConstants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, GrabberConstants.LEFT_SOLENOID_FORWARD, GrabberConstants.LEFT_SOLENOID_REVERSE);
    private DoubleSolenoid rightSolenoid = new DoubleSolenoid(GrabberConstants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, GrabberConstants.RIGHT_SOLENOID_FORWARD, GrabberConstants.RIGHT_SOLENOID_REVERSE);
    private GrabberSubsystem grabberSubsystem = new GrabberSubsystem(leftSolenoid, rightSolenoid, ledSubsystem);

    private CANSparkMax elevatorMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevatorMotor);

    private final LevelChargingStationCommand levelChargingStationCommand = new LevelChargingStationCommand(odometry, swerveDrive, ledSubsystem);
    private final AlignByAprilTag alignAtAprilTag = new AlignByAprilTag(swerveDrive, limelight, odometry, 0, -0.77);
    private final AlignByAprilTag alignLeftOfAprilTag = new AlignByAprilTag(swerveDrive, limelight, odometry, 0.64, -0.77);
    private final AlignByAprilTag alignRightOfAprilTag = new AlignByAprilTag(swerveDrive, limelight, odometry, -0.64, -0.77);


    private HashMap<String, Command> eventMap = new HashMap<>();

    private final ChargeCommand chargeCommand = new ChargeCommand(swerveDrive, odometry, eventMap);
    private final LeftUpperConeAutonomous leftUpperConeAutonomousBlue = new LeftUpperConeAutonomous(Alliance.Blue, grabberSubsystem, pivotSubsystem, telescopeSubsystem, swerveDrive, odometry, eventMap);
    private final LeftUpperConeAutonomous leftUpperConeAutonomousRed = new LeftUpperConeAutonomous(Alliance.Red, grabberSubsystem, pivotSubsystem, telescopeSubsystem, swerveDrive, odometry, eventMap);
    private final RightUpperConeAutonomous rightUpperConeAutonomous = new RightUpperConeAutonomous(grabberSubsystem, pivotSubsystem, telescopeSubsystem, swerveDrive, odometry, eventMap);
    private final MiddleUpperConeAutonomous middleAutonomous = new MiddleUpperConeAutonomous(grabberSubsystem, pivotSubsystem, telescopeSubsystem, elevatorSubsystem, swerveDrive, ledSubsystem, odometry, eventMap);
    private final MiddleAutonomousNoCone middleAutonomousNoCone = new MiddleAutonomousNoCone(grabberSubsystem, pivotSubsystem, telescopeSubsystem, elevatorSubsystem, swerveDrive, ledSubsystem, odometry, eventMap);
    private final MiddleAutonomousMiddleCone middleAutonomousMiddleCone = new MiddleAutonomousMiddleCone(grabberSubsystem, pivotSubsystem, telescopeSubsystem, elevatorSubsystem, swerveDrive, ledSubsystem, odometry, eventMap);
    private final UpperConeAutonomous upperConeAutonomous = new UpperConeAutonomous(pivotSubsystem, telescopeSubsystem, grabberSubsystem, elevatorSubsystem);

    // SENDABLE CHOOSER
    private final SendableChooser<Command> chooser = new SendableChooser<>();


    SlewRateLimiter limitX = new SlewRateLimiter(6);
    SlewRateLimiter limitY = new SlewRateLimiter(6);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(RobotBase robot) {
      this.robot = robot;

      chooser.addOption("Command", chargeCommand);
      chooser.addOption("BLUE Left Upper Cone Autonomous", leftUpperConeAutonomousBlue);
      chooser.addOption("RED Left Upper Cone Autonomous", leftUpperConeAutonomousRed);
      chooser.addOption("Right Upper Cone Autonomous", rightUpperConeAutonomous);
      chooser.addOption("Middle Autonomous Upper Cone", middleAutonomous);
      chooser.addOption("Middle Autonomous No Cone", middleAutonomousNoCone);
      chooser.addOption("Middle Autonomous Middle Cone", middleAutonomousMiddleCone);
      chooser.addOption("Just Upper Cone Autonomous :(", upperConeAutonomous);
      chooser.setDefaultOption("Middle Autonomous", middleAutonomous);
      SmartDashboard.putData(chooser);

      inclineAngle.setDefaultDouble(0);
      inclineDirection.setDefaultDouble(0);
      pitch.setDefaultDouble(0);
      roll.setDefaultDouble(0);

      // Configure the button bindings
      configureButtonBindings();
      configureObjects();
      buildAutoEventMap();
    }

    public void configureObjects() {

      frontRightSpeedMotor.setInverted(false);
      frontLeftSpeedMotor.setInverted(true);
      backRightSpeedMotor.setInverted(true);
      backLeftSpeedMotor.setInverted(false);

      frontLeftWheel.getEncoder().setInverted(true);
      frontRightWheel.getEncoder().setInverted(true);
      backLeftWheel.getEncoder().setInverted(true);
      backRightWheel.getEncoder().setInverted(true);


      frontLeftAngleMotor.setInverted(false);
      frontRightAngleMotor.setInverted(false);
      backLeftAngleMotor.setInverted(false);
      backRightAngleMotor.setInverted(false);

      backRightWheel.getEncoder().setZeroOffset(DrivetrainConstants.BACK_RIGHT_ENCODER_OFFSET);
      backLeftWheel.getEncoder().setZeroOffset(DrivetrainConstants.BACK_LEFT_ENCODER_OFFSET);
      frontRightWheel.getEncoder().setZeroOffset(DrivetrainConstants.FRONT_RIGHT_ENCODER_OFFSET);
      frontLeftWheel.getEncoder().setZeroOffset(DrivetrainConstants.FRONT_LEFT_ENCODER_OFFSET);

      elevatorMotor.setInverted(true);
      telescopeMotor.setInverted(true);
      pivotMotor.setIdleMode(IdleMode.kBrake);
      elevatorMotor.setIdleMode(IdleMode.kBrake);
      telescopeMotor.setNeutralMode(NeutralMode.Brake);

      coastDrive();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      Joystick leftJoystick = new Joystick(0);
      Joystick rightJoystick = new Joystick(1);
      Joystick altJoystick = new Joystick(2);

      

      JoystickButton balanceChargingStationButton = new JoystickButton(leftJoystick, 1);
      JoystickButton turtleButton = new JoystickButton(rightJoystick, 1);
      JoystickButton fieldCentricButton = new JoystickButton(rightJoystick, 2);

      JoystickButton pivotUpButton = new JoystickButton(altJoystick, 4);
      JoystickButton pivotDownButton = new JoystickButton(altJoystick, 1);

      JoystickButton telescopeOutButton = new JoystickButton(altJoystick, 2);
      JoystickButton telescopeInButton = new JoystickButton(altJoystick, 3);
      JoystickButton telescopeSubstation = new JoystickButton(altJoystick, 8);

      JoystickButton elevatorUpButton = new JoystickButton(altJoystick, 6);
      JoystickButton elevatorDownButton = new JoystickButton(altJoystick, 5);

      JoystickButton toggleGrabberButton = new JoystickButton(altJoystick, 9);

      Trigger pivotToTopPegButton = new Trigger(() -> altJoystick.getPOV() == 0);
      Trigger pivotToBottomButton = new Trigger(() -> altJoystick.getPOV() == 180);

      Trigger telescopeToOuterButton = new Trigger(() -> altJoystick.getPOV() == 90);
      Trigger telescopeToMiddleButton = new Trigger(() -> altJoystick.getPOV() == 270);
      JoystickButton telescopeToInButton = new JoystickButton(altJoystick, 7);

      JoystickButton alignRightOfAprilTagButton = new JoystickButton(leftJoystick, 4);
      JoystickButton alignAtAprilTagButton = new JoystickButton(leftJoystick, 2);
      JoystickButton alignLeftOfAprilTagButton = new JoystickButton(leftJoystick, 3);

      JoystickButton resetGyroButton = new JoystickButton(rightJoystick, 8);

      swerveDrive.setDefaultCommand(new RunCommand(
          () -> {
            if (robot.isTeleopEnabled()) {
              swerveDrive.drive(
                  limitX.calculate(applyDeadband(-leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND) * DriverConstants.speedMultiplier),
                  limitY.calculate(applyDeadband(-leftJoystick.getX(),DrivetrainConstants.DRIFT_DEADBAND) * DriverConstants.speedMultiplier),
                  applyDeadband(-rightJoystick.getX() * DriverConstants.angleMultiplier, DrivetrainConstants.ROTATION_DEADBAND));
            } else {
              swerveDrive.drive(0, 0, 0);
            }
          },
          swerveDrive));

      fieldCentricButton.onTrue(new InstantCommand(
          () -> {
            System.out.println("FIELD CENTRIC TOGGLED");
            swerveDrive.toggleFieldCentric();
          }, swerveDrive));
      
      balanceChargingStationButton.whileTrue(levelChargingStationCommand);

      turtleButton.whileTrue(new RunCommand(() -> swerveDrive.turtle(), swerveDrive));

      pivotUpButton.whileTrue(new RunCommand(() -> {
        pivotSubsystem.setPivotMotor(0.2);
        updatePivotTarget();
      }, pivotSubsystem));
      pivotUpButton.onFalse(new InstantCommand(() -> pivotSubsystem.setPivotPosition(pivotSubsystem.getPivotPosition()), pivotSubsystem));

      pivotDownButton.whileTrue(new RunCommand(() -> {
        pivotSubsystem.setPivotMotorVoltage(-0.1);
        updatePivotTarget();
      }, pivotSubsystem));

      elevatorUpButton.whileTrue(new RunCommand(() -> elevatorSubsystem.setElevatorMotor(0.9), elevatorSubsystem));
      elevatorUpButton.onFalse(new InstantCommand(() -> elevatorSubsystem.setElevatorMotor(0), elevatorSubsystem));

      elevatorDownButton.whileTrue(new RunCommand(() -> {
        elevatorSubsystem.setElevatorMotor(-0.9);
      }, elevatorSubsystem));
      elevatorDownButton.onFalse(new InstantCommand(() -> elevatorSubsystem.setElevatorMotor(0), elevatorSubsystem));

      telescopeOutButton.whileTrue(new RunCommand(() -> {
        if(telescopeSubsystem.getTelescopePosition() < ArmConstants.telescopeOuterSetpoint)
          telescopeSubsystem.setTelescopeMotor(0.6);
        else
          telescopeSubsystem.setTelescopeMotor(0);
      }, telescopeSubsystem));
      telescopeOutButton.onFalse(new InstantCommand(() -> telescopeSubsystem.setTelescopeMotor(0), telescopeSubsystem));

      telescopeInButton.whileTrue(new RunCommand(() -> telescopeSubsystem.setTelescopeMotor(-0.6), telescopeSubsystem));
      telescopeInButton.onFalse(new InstantCommand(() -> telescopeSubsystem.setTelescopeMotor(0), telescopeSubsystem));

      telescopeSubstation.whileTrue(new RunCommand(() -> telescopeSubsystem.setTelescopePosition(ArmConstants.telescopeSubstationSetpoint), telescopeSubsystem));

      toggleGrabberButton.onTrue(new InstantCommand(() -> grabberSubsystem.toggle(), grabberSubsystem));

      pivotToTopPegButton.onTrue(new InstantCommand(() -> pivotSubsystem.setTargetPivot(0.01), pivotSubsystem));

      pivotToBottomButton.onTrue(new InstantCommand(() -> pivotSubsystem.setTargetPivot(ArmConstants.pivotDownPosition), pivotSubsystem));

      telescopeToOuterButton.onTrue(new RunCommand(() -> {
        telescopeSubsystem.setTelescopePosition(ArmConstants.telescopeOuterSetpoint);
      }, telescopeSubsystem));

      telescopeToMiddleButton.onTrue(new RunCommand(() -> {
        telescopeSubsystem.setTelescopePosition(ArmConstants.telescopeMiddleSetpoint);
      }, telescopeSubsystem));

      telescopeToInButton.onTrue(new RunCommand(() -> {
        telescopeSubsystem.setTelescopePosition(0.06);
      }, telescopeSubsystem));

      pivotSubsystem.setDefaultCommand(new RunCommand(() -> {
        pivotSubsystem.setPivotPosition(pivotSubsystem.getTargetPivot());
      }, pivotSubsystem));

      alignAtAprilTagButton.whileTrue(alignAtAprilTag);
      alignLeftOfAprilTagButton.whileTrue(alignLeftOfAprilTag);
      alignRightOfAprilTagButton.whileTrue(alignRightOfAprilTag);
      
      resetGyroButton.onTrue(new InstantCommand(() -> {
        odometry.zeroHeading();
        swerveDrive.resetLockRot();
        odometry.reset(new Pose2d(odometry.getPose().getX(), odometry.getPose().getY(), Rotation2d.fromDegrees(180)));
      }, swerveDrive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // An ExampleCommand will run in autonomous
      return chooser.getSelected();
    }

    public double applyDeadband(double val, double deadband){
      if (Math.abs(val) < deadband) return 0;
      else return val;
    }

    public Odometry getOdometry() {
      return odometry;
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
      return swerveDrive;
    }

    public void coastDrive() {
      frontLeftAngleMotor.setIdleMode(IdleMode.kCoast);
      frontRightAngleMotor.setIdleMode(IdleMode.kCoast);
      backLeftAngleMotor.setIdleMode(IdleMode.kCoast);
      backRightAngleMotor.setIdleMode(IdleMode.kCoast);

      frontLeftSpeedMotor.setIdleMode(IdleMode.kCoast);
      frontRightSpeedMotor.setIdleMode(IdleMode.kCoast);
      backLeftSpeedMotor.setIdleMode(IdleMode.kCoast);
      backRightSpeedMotor.setIdleMode(IdleMode.kCoast);
    }

    public void periodic() {
      inclineAngle.setDouble(odometry.getInclineAngle().getDegrees());
      inclineDirection.setDouble(odometry.getInclineDirection().getDegrees());

      pitch.setDouble(odometry.getPitch().getDegrees());
      roll.setDouble(odometry.getRoll().getDegrees());
    }

    public void buildAutoEventMap(){

      eventMap.put("pickUpBall",
        new SequentialCommandGroup(
          new InstantCommand(() -> grabberSubsystem.grab()),
          new WaitCommand(0.1),
          new RunCommand(() -> pivotSubsystem.setPivotPosition(0.995))));

      eventMap.put("telescopeOut",
        new RunCommand(() -> telescopeSubsystem.setTelescopePosition(ArmConstants.telescopeOuterSetpoint)));
    }

    public void zeroSuperstructure() {
      elevatorSubsystem.setElevatorMotor(0);
      telescopeSubsystem.setTelescopeMotor(0);
      pivotSubsystem.setPivotMotor(0);

    }

    public void updatePivotTarget(){
      pivotSubsystem.setTargetPivot(pivotSubsystem.getPivotPosition());
    }
}
