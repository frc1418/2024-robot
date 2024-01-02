// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static class DrivetrainConstants {

        public static final double DRIFT_DEADBAND = 0.1;
        public static final double ROTATION_DEADBAND = 0.006;

        public static final double WIDTH = 22.5;
        public static final double LENGTH = 22.5;

        public final static int BACK_RIGHT_SPEED_ID = 4;
        public final static int BACK_RIGHT_ANGLE_ID = 3;
        public final static int BACK_RIGHT_ENCODER = 0;
        public final static double BACK_RIGHT_ENCODER_OFFSET = 0.829 - 0.25;

        public final static int BACK_LEFT_SPEED_ID = 2;
        public final static int BACK_LEFT_ANGLE_ID = 1;
        public final static int BACK_LEFT_ENCODER = 1;
        public final static double BACK_LEFT_ENCODER_OFFSET = 1-0.512;

        public final static int FRONT_RIGHT_SPEED_ID = 6;
        public final static int FRONT_RIGHT_ANGLE_ID = 5;
        public final static int FRONT_RIGHT_ENCODER = 2;
        public final static double FRONT_RIGHT_ENCODER_OFFSET = 0.142;

        public final static int FRONT_LEFT_SPEED_ID = 8;
        public final static int FRONT_LEFT_ANGLE_ID = 7;
        public final static int FRONT_LEFT_ENCODER = 3;
        public final static double FRONT_LEFT_ENCODER_OFFSET = 0.082 + 0.25;

        public final static Translation2d FRONT_LEFT_LOC = new Translation2d(0.265, 0.265);
        public final static Translation2d FRONT_RIGHT_LOC = new Translation2d(0.265, -0.265);
        public final static Translation2d BACK_LEFT_LOC = new Translation2d(-0.265, 0.265);
        public final static Translation2d BACK_RIGHT_LOC = new Translation2d(-0.265, -0.265);

        public final static SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            FRONT_LEFT_LOC,
            FRONT_RIGHT_LOC,
            BACK_LEFT_LOC,
            BACK_RIGHT_LOC);
    }

    public final static class WheelConstants {
        public final static double ROTATIONS_TO_METERS = 0.245/5.08;
    }

    public final static class ArmConstants {
        public final static int PIVOT_MOTOR_ID = 10;
        public final static int TELESCOPE_MOTOR_ID = 11;
        public final static double pivotOffset = 0.687;

        public final static double startingPivotG = 0.8;//2.01;
        public final static double pivotGPerTelescopeMeter = 0.8;
        public final static double telescopeRotationToMeters = -0.09 / 32768;

        public static final int CLICKS_PER_ROTAITON = 2048;

        public final static double telescopeOuterSetpoint = 0.9;
        public final static double telescopeMiddleSetpoint = 0.3;
        public final static double telescopeSubstationSetpoint = 0.568;

        public static double elevatorUpPivotDownPosition = 0.81;
        public static double pivotDownPosition = 0.83;
    }

    public final static class GrabberConstants {
        public final static int PNEUMATICS_HUB_ID = 23;
        public final static int LEFT_SOLENOID_REVERSE = 3;
        public final static int LEFT_SOLENOID_FORWARD = 2;

        public final static int RIGHT_SOLENOID_REVERSE = 1;
        public final static int RIGHT_SOLENOID_FORWARD = 0;
    }

    public final static class ElevatorConstants {
        public final static int ELEVATOR_MOTOR_ID = 9;
    }

    public final static class DriverConstants {
        public final static double speedMultiplier = 3;
        public final static double angleMultiplier = 3;
    }

    public final static class AutoConstants {
        public final static double autoPivotDown = 0.75;
        public final static double autoPivotUp = 0.015;
        public final static double pivotDeadband = 0.012;

        public final static double autoTelescopeIn = 0.45;
        public final static double autoTelescopeWait = 0.6;
        public final static double telescopeDeadband = 0.05;
    }

    public final static class LedConstants {
        public final static int BLINKIN_CHANNEL = 0;
    }

    public enum LedColor {
        BALANCING(0.57), //PINK
        DOCKED(0.77), //GREEN
        GRABBER_OPEN(0.67), //GOLD
        GRABBER_CLOSED(0.93), //WHITE 
        BLUE_ALLIANCE(0.87), //BLUE
        RED_ALLIANCE(0.61); //RED

        private double color;

        LedColor(double color) {
            this.color = color;
        }
        
        public double color() {
            return color;
        }

    }

    public enum LimelightDirections {
        GRID_SIDE(180), SUBSTATION_SIDE(0);

        private int angle;
        LimelightDirections(int angle){
            this.angle = angle;
        }

        public int angle(){
            return angle;
        }

    }
}
