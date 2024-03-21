// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AlignByAprilTag;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.ShootOnceCommand;
import frc.robot.commands.ShootTwiceCommand;
import frc.robot.commands.autonomous.BlueLeftOneCommand;
import frc.robot.commands.autonomous.BlueRightOneCommand;
import frc.robot.commands.autonomous.ChargeCommand;
import frc.robot.commands.autonomous.MiddleOneNoteCommand;
import frc.robot.commands.autonomous.MiddleTwoNoteCommand;
import frc.robot.commands.autonomous.ShootCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.MaxWheelModule;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimberSubsystem;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final SendableChooser<Command> autoChooser;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final RobotBase robot;

    private boolean inAuto = false;

    //Constructing the swerve wheel modules

    private CANSparkMax backRightAngleMotor = new CANSparkMax(DrivetrainConstants.BACK_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backRightSpeedMotor = new CANSparkMax(DrivetrainConstants.BACK_RIGHT_SPEED_ID, MotorType.kBrushless);
    // private AnalogEncoder backRightEncoder = new AnalogEncoder(DrivetrainConstants.BACK_RIGHT_ENCODER);

    private CANSparkMax frontRightAngleMotor = new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontRightSpeedMotor = new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_SPEED_ID, MotorType.kBrushless);
    // private AnalogEncoder frontRightEncoder = new AnalogEncoder(DrivetrainConstants.FRONT_RIGHT_ENCODER);

    private CANSparkMax backLeftAngleMotor = new CANSparkMax(DrivetrainConstants.BACK_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backLeftSpeedMotor = new CANSparkMax(DrivetrainConstants.BACK_LEFT_SPEED_ID, MotorType.kBrushless);
    // private AnalogEncoder backLeftEncoder = new AnalogEncoder(DrivetrainConstants.BACK_LEFT_ENCODER);

    private CANSparkMax frontLeftAngleMotor = new CANSparkMax(DrivetrainConstants.FRONT_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontLeftSpeedMotor = new CANSparkMax(DrivetrainConstants.FRONT_LEFT_SPEED_ID, MotorType.kBrushless);
    // private AnalogEncoder frontLeftEncoder = new AnalogEncoder(DrivetrainConstants.FRONT_LEFT_ENCODER);

    private MaxWheelModule backRightWheel = new MaxWheelModule (
        backRightAngleMotor, backRightSpeedMotor);
    public MaxWheelModule backLeftWheel = new MaxWheelModule (
      backLeftAngleMotor, backLeftSpeedMotor);
    private MaxWheelModule frontRightWheel = new MaxWheelModule (
      frontRightAngleMotor, frontRightSpeedMotor);
    private MaxWheelModule frontLeftWheel = new MaxWheelModule (
      frontLeftAngleMotor, frontLeftSpeedMotor);

    AHRS gyro = new AHRS(SPI.Port.kMXP);
    
     private SwerveModulePosition[] positions = new SwerveModulePosition[] {
      frontLeftWheel.getSwerveModulePosition(),
      frontRightWheel.getSwerveModulePosition(),
      backLeftWheel.getSwerveModulePosition(),
      backRightWheel.getSwerveModulePosition()
    };

    //Constructing the shooter subsystem
    private CANSparkMax leftShooter = new CANSparkMax(ShooterConstants.LEFT_SHOOTER_ID, MotorType.kBrushless);
    private CANSparkMax rightShooter = new CANSparkMax(ShooterConstants.RIGHT_SHOOTER_ID, MotorType.kBrushless);
    private ShooterSubsystem shooter = new ShooterSubsystem(leftShooter, rightShooter);

    //Configuring the feed subsystem
    private CANSparkMax feedMotor = new CANSparkMax(ShooterConstants.TOP_SHOOTER_ID, MotorType.kBrushless);
    private FeedSubsystem feedSubsystem = new FeedSubsystem(feedMotor);

    //Constructing the pivot subsystem
    private CANSparkMax pivotMotor = new CANSparkMax(ShooterConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    PivotSubsystem pivotSubsystem = new PivotSubsystem(pivotMotor);

    //Constructing the intake subsystem
    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeMotor);


    // Constructing the climbing subsystem
    private CANSparkMax leftClimbMotor = new CANSparkMax(ClimbConstants.LEFT_CLIMB_ID, MotorType.kBrushless);
    private CANSparkMax rightClimbMotor = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_ID, MotorType.kBrushless);
    private ClimberSubsystem climberSubsystem = new ClimberSubsystem(leftClimbMotor, rightClimbMotor);

    private SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, gyro.getRotation2d(), positions);

    private LimelightSubsystem limelight = new LimelightSubsystem();

    private Odometry odometry = new Odometry(gyro, driveOdometry, positions, limelight);

    private SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(
      backRightWheel, backLeftWheel, frontRightWheel, frontLeftWheel,
      DrivetrainConstants.SWERVE_KINEMATICS, odometry, this);

    private final AlignByAprilTag alignAtAmpCenter = new AlignByAprilTag(swerveDrive, limelight, odometry, 0.03, -0.57, 0.9, 0.07, 0.1, 90, 0);
    private final AlignByAprilTag alignAtSpeakerCenter = new AlignByAprilTag(swerveDrive, limelight, odometry, 0.03, -1.33, 1, 0.04, 0.1, 0, 0);
    // private final AlignByAprilTag alignRightOfSpeaker = new AlignByAprilTag(swerveDrive, limelight, odometry,  1.08, -0.96, 1, 0.04, 0.1, 44, 44);
    private final AlignByAprilTag alignFarFromSpeakerCenter = new AlignByAprilTag(swerveDrive, limelight, odometry, -0.06, -2.40, 1, 0.04, 0.1, 0, 0);

    //Auto Commands:
    private final ShootOnceCommand shootOnceCommand = new ShootOnceCommand(shooter, feedSubsystem);
    private final ShootTwiceCommand shootTwiceCommand = new ShootTwiceCommand(shooter, feedSubsystem, intakeSubsystem);
    private final IntakeNoteCommand intakeNoteCommand = new IntakeNoteCommand(intakeSubsystem, feedSubsystem);

    SlewRateLimiter limitX = new SlewRateLimiter(6);
    SlewRateLimiter limitY = new SlewRateLimiter(6);
    //Limits shooter motor speed
    SlewRateLimiter limitI = new SlewRateLimiter(0.5);
    //Limits intake motor speed
    SlewRateLimiter limitS = new SlewRateLimiter(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotBase robot) {
    this.robot  = robot;

    CameraServer.startAutomaticCapture();

    //Auto Named Commands:
    NamedCommands.registerCommand("intake", intakeNoteCommand);
    NamedCommands.registerCommand("shootOnce", shootOnceCommand);
    NamedCommands.registerCommand("shootTwice", shootTwiceCommand);
    NamedCommands.registerCommand("armPos", setArmCommand());
    NamedCommands.registerCommand("backFeed", backFeedCommand());

    //Auto Paths:
    final ChargeCommand chargeCommand;
    final MiddleOneNoteCommand middle1Command;
    final MiddleTwoNoteCommand middle2Command;
    final BlueLeftOneCommand blueLeftCommand;
    final BlueRightOneCommand blueRightCommand;
    final ShootCommand shootCommand;

    // Configure the trigger bindings
    configureBindings();
    //Configure the motors and sensors
    configureObjects();

    chargeCommand = new ChargeCommand(swerveDrive, feedSubsystem);
    middle1Command = new MiddleOneNoteCommand(swerveDrive, feedSubsystem);
    middle2Command = new MiddleTwoNoteCommand(swerveDrive, feedSubsystem, shooter);
    blueLeftCommand = new BlueLeftOneCommand(swerveDrive, feedSubsystem);
    blueRightCommand = new BlueRightOneCommand(swerveDrive, feedSubsystem);
    shootCommand = new ShootCommand(swerveDrive, feedSubsystem);

     // Build an auto chooser. This will use Commands.none() as the default option.
     autoChooser = AutoBuilder.buildAutoChooser();
     autoChooser.setDefaultOption("Default Path", null);
     autoChooser.addOption("Charge Command", chargeCommand);
     autoChooser.addOption("Middle 1 Note Command", middle1Command);
     autoChooser.addOption("Middle 2 Note Command", middle2Command);
     autoChooser.addOption("Blue Left Command", blueLeftCommand);
     autoChooser.addOption("Blue Right Command", blueRightCommand);
     autoChooser.addOption("Shoot Command", shootCommand);
     SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  public void configureObjects() {
    //Configuring the swerve modules
    frontLeftWheel.getTurningEncoder().setInverted(true);
    frontRightWheel.getTurningEncoder().setInverted(true);
    backLeftWheel.getTurningEncoder().setInverted(true);
    backRightWheel.getTurningEncoder().setInverted(true);

    backRightWheel.getTurningEncoder().setZeroOffset(DrivetrainConstants.BACK_RIGHT_ENCODER_OFFSET);
    backLeftWheel.getTurningEncoder().setZeroOffset(DrivetrainConstants.BACK_LEFT_ENCODER_OFFSET);
    frontRightWheel.getTurningEncoder().setZeroOffset(DrivetrainConstants.FRONT_RIGHT_ENCODER_OFFSET);
    frontLeftWheel.getTurningEncoder().setZeroOffset(DrivetrainConstants.FRONT_LEFT_ENCODER_OFFSET);

    //Configuring shooter motors
    leftShooter.setInverted(true);
    feedMotor.setInverted(true);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setIdleMode(IdleMode.kBrake);

    coastDrive();
  }

  private void configureBindings() {
    //Constructs input devices
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick altJoystick = new Joystick(2);

    JoystickButton resetFieldCentricButton = new JoystickButton(rightJoystick, 14);
    JoystickButton fieldCentricButton = new JoystickButton(leftJoystick, 1);
    JoystickButton turtleButton = new JoystickButton(rightJoystick, 16);

    JoystickButton feedOutButton = new JoystickButton(altJoystick, 2);
    JoystickButton feedInButton = new JoystickButton(altJoystick, 1);
      
    JoystickButton shooterButtonFast = new JoystickButton(altJoystick, 6);
    JoystickButton shooterButtonSlow = new JoystickButton(altJoystick, 9);

    JoystickButton pivotUpButton = new JoystickButton(altJoystick, 4);
    JoystickButton pivotDownButton = new JoystickButton(altJoystick, 3);

    //altJoystick presets:
    //0 is TOP!

    Trigger pivotAllDownPreset = new Trigger(() -> altJoystick.getPOV() == 180);
    Trigger pivotFarSpeakerPreset = new Trigger(() -> altJoystick.getPOV() == 270);
    Trigger pivotAmpPreset = new Trigger(() -> altJoystick.getPOV() == 0);
    Trigger pivotCloseSpeakerPreset = new Trigger(() -> altJoystick.getPOV() == 90);

    //  Trigger climberUp = new Trigger(() -> leftJoystick.getPOV() == 0);
    //  Trigger climberDown = new Trigger(() -> leftJoystick.getPOV() == 180);
    //  Trigger climberStop = new Trigger(() -> leftJoystick.getPOV() == -1);

    JoystickButton intakeButton = new JoystickButton(altJoystick, 5);
    JoystickButton intakeOutButton =  new JoystickButton(altJoystick, 7);
  
    JoystickButton alignAtAmpCenterButton = new JoystickButton(rightJoystick, 2);
    JoystickButton alignAtSpeakerCenterButton = new JoystickButton(rightJoystick, 3);
    // JoystickButton alignRightOfSpeakerButton = new JoystickButton(rightJoystick, 3);
    JoystickButton alignFarFromSpeakerButton = new JoystickButton(rightJoystick, 4);

    JoystickButton climbUpButton = new JoystickButton(leftJoystick, 3);
    JoystickButton climbDownButton = new JoystickButton(leftJoystick, 4);

    //Constructs commands and binds them for swerve drive
    swerveDrive.setDefaultCommand(new RunCommand(() -> {
      if (robot.isTeleopEnabled()){
        if (swerveDrive.getFieldCentric()) {
          swerveDrive.drive(
            -limitX.calculate(applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
            -limitY.calculate(applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
            applyDeadband(-rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND)*DriverConstants.angleMultiplier);
        }
        else {
          swerveDrive.drive(
            limitX.calculate(applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
            limitY.calculate(applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
            applyDeadband(-rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND)*DriverConstants.angleMultiplier);
        }
      }
      else 
      {
        swerveDrive.drive(0,0,0);
      }
      
    }, swerveDrive));

    fieldCentricButton.onTrue(swerveDrive.robotCentricCommand());
    fieldCentricButton.onFalse(swerveDrive.fieldCentricCommand());

    resetFieldCentricButton.onTrue(swerveDrive.resetFieldCentric());

    turtleButton.whileTrue(new RunCommand(() -> {
      swerveDrive.turtle();
    }, swerveDrive));

    //Constructs commands and binds them for shooter

    shooter.setDefaultCommand(new RunCommand(() -> {
      shooter.shoot(limitS.calculate(0));
    }, shooter));

    shooterButtonFast.whileTrue(new RunCommand(() -> {
      shooter.shoot(limitS.calculate(applyDeadband(1.0, ShooterConstants.SHOOTER_DEADBAND)));  
    }, shooter));

    shooterButtonSlow.whileTrue(new RunCommand(() -> {
      shooter.shoot(limitS.calculate(applyDeadband(0.3, ShooterConstants.SHOOTER_DEADBAND)));  
    }, shooter));

    //Constructs commands and binds them for feed

    feedSubsystem.setDefaultCommand(new RunCommand(() -> {
      feedSubsystem.feed(0);
    }, feedSubsystem));

    feedInButton.whileTrue(new RunCommand(() -> {
      feedSubsystem.feed(0.25);
    }, feedSubsystem));

    feedOutButton.whileTrue(new RunCommand(() -> {
      feedSubsystem.feed(-0.1);
    }, feedSubsystem));

    //Constructs commands and binds them for pivot

    pivotUpButton.onTrue(new InstantCommand(() -> {
      pivotSubsystem.changeLockPos(-0.005);
    }));

    pivotDownButton.onTrue(new InstantCommand(() -> {
      pivotSubsystem.changeLockPos(0.005);
    }));

    // Constructs commands and binds them for climber
    climberSubsystem.setDefaultCommand(new RunCommand(() -> {
      climberSubsystem.stopClimbing();
    }, climberSubsystem));

    climbDownButton.whileTrue(new RunCommand(() -> {
      climberSubsystem.climb(-0.5);
    }, climberSubsystem));

    climbUpButton.whileTrue(new RunCommand(() -> {
      climberSubsystem.climb(0.5);
    }, climberSubsystem));

    //Constructs commands and binds them for intake

    intakeSubsystem.setDefaultCommand(new RunCommand(() -> {
      intakeSubsystem.intake(0);
    }, intakeSubsystem));

    intakeButton.whileTrue(new RunCommand(() -> {
      intakeSubsystem.intake(limitI.calculate(0.5));
      feedSubsystem.feed(0.25);
      pivotSubsystem.setLockPos(0.82);
    }, intakeSubsystem, feedSubsystem));

    intakeOutButton.whileTrue(new RunCommand(() -> {
      intakeSubsystem.intake(limitI.calculate(-0.5));
      feedSubsystem.feed(-0.25);
    }, intakeSubsystem, feedSubsystem));

    pivotAllDownPreset.onTrue(new InstantCommand(() -> {
      pivotSubsystem.setLockPos(0.999);
    }));

    pivotFarSpeakerPreset.onTrue(new InstantCommand(() -> {
      pivotSubsystem.setLockPos(0.8999);
    }));

    pivotAmpPreset.onTrue(new InstantCommand(() -> {
      pivotSubsystem.setLockPos(0.8956);
    }));

    pivotCloseSpeakerPreset.onTrue(new InstantCommand(() -> {
      pivotSubsystem.setLockPos(0.8655);
    }));

    
    //Constructs commands and binds them for AprilTags
    alignAtAmpCenterButton.whileTrue(alignAtAmpCenter);
    alignAtSpeakerCenterButton.whileTrue(alignAtSpeakerCenter);
    // alignRightOfSpeakerButton.whileTrue(alignRightOfSpeaker);
    alignFarFromSpeakerButton.whileTrue(alignFarFromSpeakerCenter);
  }

  public double applyDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) 
      return 0;
    else return 
      input;
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

  public Odometry getOdometry() {
    return odometry;
  }

  public SwerveDriveSubsystem getSwerveDriveSubsystem() {
    return swerveDrive;
  }

  public void setAuto(boolean auto) {
    inAuto = auto;
  }

  public boolean getAuto() {
    return inAuto;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   
   public Command backFeedCommand()
   {
    return (new InstantCommand(() -> {
      feedSubsystem.feed(-0.4);
      System.out.println("backFeed");
    }));
   }

   public Command setArmCommand()
   {
    return (new RunCommand(() -> {
      pivotSubsystem.setLockPos(0.855);
    }));
   }

   public Command getAutonomousCommand () {
      return autoChooser.getSelected();
  }
}
