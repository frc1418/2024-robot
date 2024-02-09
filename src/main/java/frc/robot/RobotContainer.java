// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.common.Odometry;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.MaxWheelModule;
import frc.robot.subsystems.ShooterSubsystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here
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

    //Constructing the intake subsystem
    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeMotor);

    //Constructing the shooter subsystem
    private CANSparkMax bottomLeftShooter = new CANSparkMax(ShooterConstants.BOTTOM_LEFT_SHOOTER_ID, MotorType.kBrushless);
    private CANSparkMax bottomRightShooter = new CANSparkMax(ShooterConstants.BOTTOM_RIGHT_SHOOTER_ID, MotorType.kBrushless);
    private CANSparkMax topLeftShooter = new CANSparkMax(ShooterConstants.TOP_LEFT_SHOOTER_ID, MotorType.kBrushless);
    private CANSparkMax topRightShooter = new CANSparkMax(ShooterConstants.TOP_RIGHT_SHOOTER_ID, MotorType.kBrushless);
    private CANSparkMax topShooter = new CANSparkMax(ShooterConstants.TOP_SHOOTER_ID, MotorType.kBrushless);
    private ShooterSubsystem shooter = new ShooterSubsystem(bottomLeftShooter, bottomRightShooter, topLeftShooter, topRightShooter, topShooter);

    private SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, gyro.getRotation2d(), positions);

    private Odometry odometry = new Odometry(gyro, driveOdometry, positions);

    private SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(
      backRightWheel, backLeftWheel, frontRightWheel, frontLeftWheel,
      DrivetrainConstants.SWERVE_KINEMATICS, odometry);

    SlewRateLimiter limitX = new SlewRateLimiter(6);
    SlewRateLimiter limitY = new SlewRateLimiter(6);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotBase robot) {
    this.robot  = robot;
    // Configure the trigger bindings
    configureBindings();
    //Configure the motors and sensors
    configureObjects();
  }
  
  public void configureObjects() {
    resetMotors();

    //Configuring the swerve modules
    frontLeftWheel.getTurningEncoder().setInverted(true);
    frontRightWheel.getTurningEncoder().setInverted(true);
    backLeftWheel.getTurningEncoder().setInverted(true);
    backRightWheel.getTurningEncoder().setInverted(true);

    backRightWheel.getTurningEncoder().setZeroOffset(DrivetrainConstants.BACK_RIGHT_ENCODER_OFFSET);
    backLeftWheel.getTurningEncoder().setZeroOffset(DrivetrainConstants.BACK_LEFT_ENCODER_OFFSET);
    frontRightWheel.getTurningEncoder().setZeroOffset(DrivetrainConstants.FRONT_RIGHT_ENCODER_OFFSET);
    frontLeftWheel.getTurningEncoder().setZeroOffset(DrivetrainConstants.FRONT_LEFT_ENCODER_OFFSET);

    frontLeftWheel.getTurningEncoder().setInverted(true);
    frontRightWheel.getTurningEncoder().setInverted(true);
    backLeftWheel.getTurningEncoder().setInverted(true);
    backRightWheel.getTurningEncoder().setInverted(true);

    //Configuring shooter motors
    topLeftShooter.setInverted(true);
    bottomRightShooter.setInverted(true);

    coastDrive();
  }

  private void configureBindings() {
    //Constructs input devices
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick altJoystick = new Joystick(2);

    JoystickButton turtleButton = new JoystickButton(rightJoystick, 1);

    JoystickButton fieldCentricButton = new JoystickButton(rightJoystick, 2);

    JoystickButton resetFieldCentricButton = new JoystickButton(leftJoystick, 2);

    JoystickButton intakeButton = new JoystickButton(leftJoystick, 1);

    JoystickButton feedInButton = new JoystickButton(rightJoystick, 3);

    JoystickButton feedOutButton = new JoystickButton(rightJoystick, 4);


    //Constructs commands and binds them
    swerveDrive.setDefaultCommand(new RunCommand(() -> {
      if (robot.isTeleopEnabled()){
        swerveDrive.drive(
          limitY.calculate(applyDeadband(-leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
          limitX.calculate(applyDeadband(-leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
          applyDeadband(-rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND)*DriverConstants.angleMultiplier);
      }
      else 
      {
        swerveDrive.drive(0,0,0);
      }
      
    }, swerveDrive));

    shooter.setDefaultCommand(new RunCommand(() -> {
      shooter.feed(0);
    }, shooter));

    intakeSubsystem.setDefaultCommand(new RunCommand(() -> {
      intakeSubsystem.intake(0);
    }, intakeSubsystem));

    fieldCentricButton.onTrue(swerveDrive.toggleFieldCentric());

    resetFieldCentricButton.onTrue(swerveDrive.resetFieldCentric());

    turtleButton.whileTrue(new RunCommand(() -> {
      swerveDrive.turtle();
    }, swerveDrive));

    intakeButton.whileTrue(new RunCommand(() -> {
      intakeSubsystem.intake(limitX.calculate((applyDeadband(-leftJoystick.getThrottle(), IntakeConstants.INTAKE_DEADBAND))));
    }, intakeSubsystem));

    feedInButton.whileTrue(new RunCommand(() -> {
      shooter.feed(0.1);
    }, shooter));

    feedOutButton.whileTrue(new RunCommand((
    ) -> {
      shooter.feed(-0.1);
    }, shooter));
  }

  public double applyDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) 
      return 0;
    else return 
      input;
  }

  public void resetMotors() {
    backLeftAngleMotor.restoreFactoryDefaults();
    backRightAngleMotor.restoreFactoryDefaults();
    frontLeftAngleMotor.restoreFactoryDefaults();
    frontRightAngleMotor.restoreFactoryDefaults();

    backLeftSpeedMotor.restoreFactoryDefaults();
    backRightSpeedMotor.restoreFactoryDefaults();
    frontLeftSpeedMotor.restoreFactoryDefaults();
    frontRightSpeedMotor.restoreFactoryDefaults();

    intakeMotor.restoreFactoryDefaults();

    bottomLeftShooter.restoreFactoryDefaults();
    bottomRightShooter.restoreFactoryDefaults();
    topLeftShooter.restoreFactoryDefaults();
    topRightShooter.restoreFactoryDefaults();
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
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  } 
}
