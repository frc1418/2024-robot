// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.Autos;
import frc.robot.common.Odometry;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MarkWheelSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.MaxWheelSubsystem;
import frc.robot.subsystems.ShooterWheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogEncoder;
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
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final RobotBase robot;

    // private CANSparkMax backRightAngleMotor = new CANSparkMax(DrivetrainConstants.BACK_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax bottomRightSpeedMotor = new CANSparkMax(DrivetrainConstants.BOTTOM_RIGHT_SPEED_ID, MotorType.kBrushless);
    // private AnalogEncoder backRightEncoder = new AnalogEncoder(DrivetrainConstants.BACK_RIGHT_ENCODER);

    // private CANSparkMax frontRightAngleMotor = new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax topRightSpeedMotor = new CANSparkMax(DrivetrainConstants.TOP_RIGHT_SPEED_ID, MotorType.kBrushless);
    // private AnalogEncoder frontRightEncoder = new AnalogEncoder(DrivetrainConstants.FRONT_RIGHT_ENCODER);

    // private CANSparkMax backLeftAngleMotor = new CANSparkMax(DrivetrainConstants.BACK_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax bottomLeftSpeedMotor = new CANSparkMax(DrivetrainConstants.BOTTOM_LEFT_SPEED_ID, MotorType.kBrushless);
    // private AnalogEncoder backLeftEncoder = new AnalogEncoder(DrivetrainConstants.BACK_LEFT_ENCODER);

    // private CANSparkMax frontLeftAngleMotor = new CANSparkMax(DrivetrainConstants.FRONT_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax topLeftSpeedMotor = new CANSparkMax(DrivetrainConstants.TOP_LEFT_SPEED_ID, MotorType.kBrushless);
    // private AnalogEncoder frontLeftEncoder = new AnalogEncoder(DrivetrainConstants.FRONT_LEFT_ENCODER);



    private ShooterWheelSubsystem bottomRightWheel = new ShooterWheelSubsystem(bottomRightSpeedMotor);
    private ShooterWheelSubsystem bottomLeftWheel = new ShooterWheelSubsystem(bottomLeftSpeedMotor);
    private ShooterWheelSubsystem topRightWheel = new ShooterWheelSubsystem(topRightSpeedMotor);
    private ShooterWheelSubsystem topLeftWheel = new ShooterWheelSubsystem(topLeftSpeedMotor);




    AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    //  private SwerveModulePosition[] positions = new SwerveModulePosition[] {
    //   frontLeftWheel.getSwerveModulePosition(),
    //   frontRightWheel.getSwerveModulePosition(),
    //   backLeftWheel.getSwerveModulePosition(),
    //   backRightWheel.getSwerveModulePosition()
    // };

    // private SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, gyro.getRotation2d(), positions);

    // private Odometry odometry = new Odometry(gyro, driveOdometry, positions);

    private ShooterSubsystem shooter = new ShooterSubsystem(bottomLeftWheel, bottomRightWheel, topLeftWheel, topRightWheel);

    // private SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(
    //   backRightWheel, backLeftWheel, frontRightWheel, frontLeftWheel,
    //   DrivetrainConstants.SWERVE_KINEMATICS, odometry);

    SlewRateLimiter limitX = new SlewRateLimiter(6);
    SlewRateLimiter limitY = new SlewRateLimiter(6);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotBase robot) {
    this.robot  = robot;
    // Configure the trigger bindings
    configureBindings();
    configureObjects();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
  public void configureObjects() {

    coastDrive();
  }

  private void configureBindings() {
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick altJoystick = new Joystick(2);

    JoystickButton turtleButton = new JoystickButton(rightJoystick, 1);

    JoystickButton fieldCentricButton = new JoystickButton(rightJoystick, 2);

    JoystickButton resetFieldCentricButton = new JoystickButton(leftJoystick, 2);


    shooter.setDefaultCommand(new RunCommand(() -> {
      if (robot.isTeleopEnabled()){
        if (leftJoystick.getThrottle() < 0) {
          shooter.shoot(
            limitY.calculate(applyDeadband(-leftJoystick.getThrottle(), DrivetrainConstants.DRIFT_DEADBAND)));
        }
        else {
          shooter.shoot(
            limitY.calculate(applyDeadband(-leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND)));        
        }
      }
      else 
      {
        shooter.shoot(0);
      }
      
    }, shooter));

    // fieldCentricButton.onTrue(new InstantCommand((
    //   ) -> {
    //     swerveDrive.toggleFieldCentric();
    //   }, swerveDrive));

    // resetFieldCentricButton.onTrue(swerveDrive.toggleFieldCentric());

    // turtleButton.whileTrue(new RunCommand((
    // ) -> {
    //   swerveDrive.turtle();
    // }, swerveDrive));  
  }

  public double applyDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) 
      return 0;
    else return 
      input;
  }

  public void coastDrive() {
    // frontLeftAngleMotor.setIdleMode(IdleMode.kCoast);
    // frontRightAngleMotor.setIdleMode(IdleMode.kCoast);
    // backLeftAngleMotor.setIdleMode(IdleMode.kCoast);
    // backRightAngleMotor.setIdleMode(IdleMode.kCoast);

    topLeftSpeedMotor.setIdleMode(IdleMode.kCoast);
    topRightSpeedMotor.setIdleMode(IdleMode.kCoast);
    bottomLeftSpeedMotor.setIdleMode(IdleMode.kCoast);
    bottomRightSpeedMotor.setIdleMode(IdleMode.kCoast);
  }

  public Odometry getOdometry() {
    return null;
  }

  public SwerveDriveSubsystem getSwerveDriveSubsystem() {
    return null;
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
