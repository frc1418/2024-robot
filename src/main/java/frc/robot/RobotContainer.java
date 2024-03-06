// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AlignByAprilTag;
import frc.robot.commands.autonomous.ChargeCommand;
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


import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  // The robot's subsystems and commands are defined here
  // private final SendableChooser<Command> autoChooser;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final RobotBase robot;

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
    private PivotSubsystem pivotSubsystem = new PivotSubsystem(pivotMotor);

    //Constructing the intake subsystem
    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeMotor);

    private CANSparkMax LeftClimbMotor = new CANSparkMax(ClimbConstants.LEFT_CLIMB_ID, MotorType.kBrushless);
    private CANSparkMax RightClimbMotor = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_ID, MotorType.kBrushless);
    private ClimberSubsystem climberSubsystem = new ClimberSubsystem(LeftClimbMotor, RightClimbMotor);

    private SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, gyro.getRotation2d(), positions);

    private LimelightSubsystem limelight = new LimelightSubsystem();

    private Odometry odometry = new Odometry(gyro, driveOdometry, positions, limelight);

    private SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(
      backRightWheel, backLeftWheel, frontRightWheel, frontLeftWheel,
      DrivetrainConstants.SWERVE_KINEMATICS, odometry);

    private final AlignByAprilTag alignAtAmpCenter = new AlignByAprilTag(swerveDrive, limelight, odometry, 0, -0.63, 0.9, 0.07, 0.1, 90, 0);
    private final AlignByAprilTag alignAtSpeakerCenter = new AlignByAprilTag(swerveDrive, limelight, odometry, 0.04, -1.4, 1, 0.04, 0.1, 0, 0);
    private final AlignByAprilTag alignRightOfSpeaker = new AlignByAprilTag(swerveDrive, limelight, odometry,  1.08, -0.96, 1, 0.04, 0.1, 44, 44);
    private final AlignByAprilTag alignLeftOfSpeaker = new AlignByAprilTag(swerveDrive, limelight, odometry,  -1.08, -0.96, 1, 0.04, 0.1, -44, -4);

    private final ChargeCommand chargeCommand;
    
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

    //Auto Test commands:
    NamedCommands.registerCommand("print", new PrintCommand("print!"));
    NamedCommands.registerCommand("printRight2NoteRed", new PrintCommand("righ2notered path!"));
    NamedCommands.registerCommand("printLeft2Note", new PrintCommand("left2note path!"));
    NamedCommands.registerCommand("printAfter", new PrintCommand("print at end!"));

    NamedCommands.registerCommand("intake", intakeNoteCommand());
    NamedCommands.registerCommand("shoot", shootNoteCommand());

    // Configure the trigger bindings
    configureBindings();
    //Configure the motors and sensors
    configureObjects();

    System.out.println("CREATING CHARGE COMMAND");
    chargeCommand = new ChargeCommand(swerveDrive, feedSubsystem);

     // Build an auto chooser. This will use Commands.none() as the default option.
    //  autoChooser = AutoBuilder.buildAutoChooser();
    //  autoChooser.setDefaultOption("Default Path", null);
    //  autoChooser.addOption("Charge Command", chargeCommand);
    //  SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  public void configureObjects() {
    // resetMotors();

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

    coastDrive();
  }

  private void configureBindings() {
    //Constructs input devices
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick altJoystick = new Joystick(2);

    JoystickButton resetFieldCentricButton = new JoystickButton(rightJoystick, 14);
    JoystickButton fieldCentricButton = new JoystickButton(rightJoystick, 15);
    JoystickButton turtleButton = new JoystickButton(rightJoystick, 16);

    JoystickButton feedOutButton = new JoystickButton(leftJoystick, 3);
    JoystickButton altFeedOutButton = new JoystickButton(altJoystick, 2);
    JoystickButton feedInButton = new JoystickButton(leftJoystick, 4);
    JoystickButton altFeedInButton = new JoystickButton(altJoystick, 1);

    JoystickButton shooterButton = new JoystickButton(rightJoystick, 1);
    JoystickButton altShooterButton = new JoystickButton(altJoystick, 6);

    JoystickButton pivotButton = new JoystickButton(leftJoystick, 2);
    // JoystickButton altPivotButton = new JoystickButton(altJoystick, 2);
    JoystickButton pivotUpButton = new JoystickButton(leftJoystick, 8);
    JoystickButton altPivotUpButton = new JoystickButton(altJoystick, 4);
    JoystickButton pivotDownButton = new JoystickButton(leftJoystick, 14);
    JoystickButton altPivotDownButton = new JoystickButton(altJoystick, 3);
    JoystickButton allDownButton = new JoystickButton(rightJoystick, 8);
    JoystickButton altAllDownButton = new JoystickButton(altJoystick, 10);
    // JoystickButton allDownButton = new JoystickButton(altJoystick, 0)

    JoystickButton intakeButton = new JoystickButton(leftJoystick, 1);
    JoystickButton altIntakeButton = new JoystickButton(altJoystick, 5);
  
    JoystickButton alignAtAmpCenterButton = new JoystickButton(rightJoystick, 2);
    // JoystickButton alignAtSpeakerCenterButton = new JoystickButton(rightJoystick, 3);
    JoystickButton alignRightOfSpeakerButton = new JoystickButton(rightJoystick, 3);
    JoystickButton alignLeftOfSpeakerButton = new JoystickButton(rightJoystick, 4);

    JoystickButton ClimbUpButton = new JoystickButton(leftJoystick, 5);
    JoystickButton ClimbDownButton = new JoystickButton(leftJoystick, 6);

    //Constructs commands and binds them for swerve drive
    swerveDrive.setDefaultCommand(new RunCommand(() -> {
      if (robot.isTeleopEnabled()){
        swerveDrive.drive(
          -limitX.calculate(applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
          -limitY.calculate(applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
          applyDeadband(-rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND)*DriverConstants.angleMultiplier);
      }
      else 
      {
        swerveDrive.drive(0,0,0);
      }
      
    }, swerveDrive));

    fieldCentricButton.onTrue(swerveDrive.toggleFieldCentric());

    resetFieldCentricButton.onTrue(swerveDrive.resetFieldCentric());

    turtleButton.whileTrue(new RunCommand(() -> {
      swerveDrive.turtle();
    }, swerveDrive));

    //Constructs commands and binds them for shooter

    shooter.setDefaultCommand(new RunCommand(() -> {
      shooter.shoot(limitS.calculate(0));
    }, shooter));

    // shooterButton.whileTrue(new RunCommand(() -> {
    //   shooter.shoot(limitS.calculate(applyDeadband(-rightJoystick.getThrottle(), ShooterConstants.SHOOTER_DEADBAND)));
    // }, shooter));

    altShooterButton.whileTrue(new RunCommand(() -> {
      shooter.shoot(limitS.calculate(applyDeadband(-rightJoystick.getThrottle(), ShooterConstants.SHOOTER_DEADBAND)));  
    }, shooter));

    //Constructs commands and binds them for feed

    feedSubsystem.setDefaultCommand(new RunCommand(() -> {
      feedSubsystem.feed(0);
    }, feedSubsystem));

    // feedInButton.whileTrue(new RunCommand(() -> {
    //   feedSubsystem.feed(0.25);
    // }, feedSubsystem));

    altFeedInButton.whileTrue(new RunCommand(() -> {
      feedSubsystem.feed(0.25);
    }, feedSubsystem));

    // feedOutButton.whileTrue(new RunCommand(() -> {
    //   feedSubsystem.feed(-0.1);
    // }, feedSubsystem));

    altFeedOutButton.whileTrue(new RunCommand(() -> {
      feedSubsystem.feed(-0.1);
    }, feedSubsystem));

    //Constructs commands and binds them for pivot

    pivotSubsystem.setDefaultCommand(new RunCommand(() -> {
      // pivotSubsystem.setPivotPosition(pivotSubsystem.getLockPos());
    }, pivotSubsystem));

    // pivotButton.whileTrue(new RunCommand(() -> {
    //   pivotSubsystem.setPivotPosition(pivotSubsystem.getTargetPos());
    //   pivotSubsystem.setLockPos(MathUtil.clamp(pivotSubsystem.getTargetPos(),0.82, 0.972));
    // }, pivotSubsystem));

    // altPivotButton.whileTrue(new RunCommand(() -> {
    //   pivotSubsystem.setPivotPosition(pivotSubsystem.getTargetPos());
    //   pivotSubsystem.setLockPos(MathUtil.clamp(pivotSubsystem.getTargetPos(),0.82, 0.972));
    // }, pivotSubsystem));

    // allDownButton.onTrue(new InstantCommand(() -> {
    //   pivotSubsystem.setPivotPosition(0.972);
    // }));

    altAllDownButton.onTrue(new InstantCommand(() -> {
      pivotSubsystem.setLockPos(0.972);
    }));

    // pivotUpButton.onTrue(new InstantCommand(() -> {
    //   pivotSubsystem.changeTargetPos(-0.01);
    // }));

    altPivotUpButton.onTrue(new InstantCommand(() -> {
      pivotSubsystem.changeLockPos(-0.01);
    }));

    // pivotDownButton.onTrue(new InstantCommand(() -> {
    //   pivotSubsystem.changeTargetPos(0.01);
    // }));

    altPivotDownButton.onTrue(new InstantCommand(() -> {
      pivotSubsystem.changeLockPos(0.01);
    }));

    //Constructs commands and binds them for climber
    ClimbDownButton.onTrue(new InstantCommand(() -> {
      climberSubsystem.climb(-0.5);
  }, climberSubsystem));
  
  ClimbDownButton.onFalse(new InstantCommand(() -> {
      climberSubsystem.stopClimbing();
  }, climberSubsystem));
  
  ClimbUpButton.onTrue(new InstantCommand(() -> {
      climberSubsystem.climb(0.5);
  }, climberSubsystem));
  
  ClimbUpButton.onFalse(new InstantCommand(() -> {
      climberSubsystem.stopClimbing();
  }, climberSubsystem));
  


    //Constructs commands and binds them for intake

    intakeSubsystem.setDefaultCommand(new RunCommand(() -> {
      intakeSubsystem.intake(0);
    }, intakeSubsystem));

    intakeButton.whileTrue(new RunCommand(() -> {
      intakeSubsystem.intake(limitI.calculate((applyDeadband(-leftJoystick.getThrottle()/2, IntakeConstants.INTAKE_DEADBAND))));
      feedSubsystem.feed(0.25);
      pivotSubsystem.setLockPos(0.82);
    }, intakeSubsystem, feedSubsystem));

    altIntakeButton.whileTrue(new RunCommand(() -> {
      intakeSubsystem.intake(limitI.calculate((applyDeadband(-leftJoystick.getThrottle()/2, IntakeConstants.INTAKE_DEADBAND))));
      feedSubsystem.feed(0.25);
      pivotSubsystem.setLockPos(0.82);
    }, intakeSubsystem, feedSubsystem));

    
    //Constructs commands and binds them for AprilTags
    alignAtAmpCenterButton.whileTrue(alignAtAmpCenter);
    // alignAtSpeakerCenterButton.whileTrue(alignAtSpeakerCenter);
    alignRightOfSpeakerButton.whileTrue(alignRightOfSpeaker);
    alignLeftOfSpeakerButton.whileTrue(alignLeftOfSpeaker);

  }

  public double applyDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) 
      return 0;
    else return 
      input;
  }

  public void resetMotors() {
    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
    feedMotor.restoreFactoryDefaults();

    pivotMotor.restoreFactoryDefaults();

    intakeMotor.restoreFactoryDefaults();
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
   public Command intakeNoteCommand()
   {
    return (new RunCommand(() -> {
    intakeSubsystem.intake(0.5);
    System.out.println("intake");
    feedSubsystem.feed(0.25);
    pivotSubsystem.setLockPos(0.82);
    }, intakeSubsystem, feedSubsystem));
   }

   public Command shootNoteCommand()
   {
    return (new RunCommand(() -> {
      shooter.shoot(0.75); 
      feedSubsystem.feed(0.25);
      pivotSubsystem.setLockPos(0.25); 
    }, shooter));
   }

   public Command getAutonomousCommand () {
      // return autoChooser.getSelected();
      return chargeCommand;
  }
}
