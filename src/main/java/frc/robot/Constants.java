// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final static class DrivetrainConstants{
    public static final int BACK_RIGHT_ANGLE_ID = 0;
    public static final int BACK_RIGHT_SPEED_ID = 0;
    public static final AnalogInput BACK_RIGHT_ENCODER = null;
    public static final Translation2d BACK_RIGHT_LOC = new Translation2d(0, 0);


    public static final int FRONT_RIGHT_ANGLE_ID = 0;
    public static final int FRONT_RIGHT_SPEED_ID = 0;
    public static final AnalogInput FRONT_RIGHT_ENCODER = null;
    public static final Translation2d FRONT_RIGHT_LOC = new Translation2d(0, 0);

    

    public static final int BACK_LEFT_ANGLE_ID = 1;
    public static final int BACK_LEFT_SPEED_ID = 0;
    public static final int BACK_LEFT_ENCODER = 1;
    public static final Translation2d BACK_LEFT_LOC = new Translation2d(0, 0);
    
    public static final int FRONT_LEFT_ANGLE_ID = 0;
    public static final int FRONT_LEFT_SPEED_ID = 0;
    public static final AnalogInput FRONT_LEFT_ENCODER = null;
    public static final Translation2d FRONT_LEFT_LOC = new Translation2d(0, 0);
    
	  public static final SwerveDriveKinematics SWERVE_KINEMATICS = null;
  }
}
