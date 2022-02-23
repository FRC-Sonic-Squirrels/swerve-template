// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.subsystems.DrivetrainSubsystem;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.0);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19.0); 

    // Set pigeon ID to -1 to disable and use NAVX on SPI.Port.kMXP
    public static final int DRIVETRAIN_PIGEON_ID = 15;
    
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(142.4 + 180);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(65.4 + 180);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 14; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(28.1 + 180);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 13; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(182.7 - 180);

    public static final class AutoConstants {
        // This kP worked for the DriveWithSetRotation command
        public static final double kPThetaController = 3.0;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.02;
    
        // Feed Forward and PID values from SysId
        public static final double kP = 2.3055;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kA = 0.12817;
        public static final double kV = 2.3423;
        public static final double kS = 0.53114;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        DrivetrainSubsystem.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    // according to game manual field is 27 ft. (~823 cm) by 54 ft. (~1646 cm)
    public static final Vector2d HUB_CENTER = new Vector2d(8.23, 4.11);
    public static final Pose2d ROBOT_1M_LEFT_OF_HUB =
            new Pose2d(HUB_CENTER.x - 1, HUB_CENTER.y, new Rotation2d(0));


    public static class HubCentricConstants{
        public static final Vector2d HUB_CENTER = new Vector2d(8.23, 4.11);
        public static final double FORWARD_MULTIPLIER = 0.5;
        public static final double SIDEWAYS_MULTIPLIER = 0.3;          
   }

   public static class FieldConstants{
        public Translation2d BLUE_CARGO_1 = new Translation2d( Units.inchesToMeters(42), Units.inchesToMeters(44.4));
        public Translation2d BLUE_CARGO_2 = new Translation2d( Units.inchesToMeters(198), Units.inchesToMeters(72));
        public Translation2d BLUE_CARGO_3 = new Translation2d( Units.inchesToMeters(297.6), Units.inchesToMeters(7.2));
        public Translation2d BLUE_CARGO_4 = new Translation2d( Units.inchesToMeters(412.8), Units.inchesToMeters(36));
        public Translation2d BLUE_CARGO_5 = new Translation2d( Units.inchesToMeters(472.8), Units.inchesToMeters(198));
        public Translation2d BLUE_CARGO_6 = new Translation2d( Units.inchesToMeters(290.4), Units.inchesToMeters(312));
        public Translation2d BLUE_CARGO_7 = new Translation2d( Units.inchesToMeters(196.8), Units.inchesToMeters(246));
        public Translation2d RED_CARGO_1 = new Translation2d( Units.inchesToMeters(605), Units.inchesToMeters(280));
        public Translation2d RED_CARGO_2 = new Translation2d( Units.inchesToMeters(257), Units.inchesToMeters(441));
        public Translation2d RED_CARGO_3 = new Translation2d( Units.inchesToMeters(350), Units.inchesToMeters(314));
        public Translation2d RED_CARGO_4 = new Translation2d( Units.inchesToMeters(235), Units.inchesToMeters(275));
        public Translation2d RED_CARGO_5 = new Translation2d( Units.inchesToMeters(174), Units.inchesToMeters(127));
        public Translation2d RED_CARGO_6 = new Translation2d( Units.inchesToMeters(357), Units.inchesToMeters(12));
        public Translation2d RED_CARGO_7 = new Translation2d( Units.inchesToMeters(460), Units.inchesToMeters(246));
        public Translation2d HUB_CENTER = new Translation2d( Units.inchesToMeters(324), Units.inchesToMeters(162));
        public Translation2d BLUE_LOW = new Translation2d( Units.inchesToMeters(130), Units.inchesToMeters(264));
        public Translation2d BLUE_MID = new Translation2d( Units.inchesToMeters(85), Units.inchesToMeters(264));
        public Translation2d BLUE_HIGH = new Translation2d( Units.inchesToMeters(62), Units.inchesToMeters(264));
        public Translation2d BLUE_TRANSVERSAL = new Translation2d( Units.inchesToMeters(38), Units.inchesToMeters(264));
        public Translation2d BLUE_PAD_1 = new Translation2d( Units.inchesToMeters(130), Units.inchesToMeters(216));
        public Translation2d BLUE_PAD_2 = new Translation2d( Units.inchesToMeters(130), Units.inchesToMeters(312));
        public Translation2d RED_LOW = new Translation2d( Units.inchesToMeters(518), Units.inchesToMeters(60));
        public Translation2d RED_MID = new Translation2d( Units.inchesToMeters(562), Units.inchesToMeters(60));
        public Translation2d RED_HIGH = new Translation2d( Units.inchesToMeters(586), Units.inchesToMeters(60));
        public Translation2d RED_TRANSVERSAL = new Translation2d( Units.inchesToMeters(607), Units.inchesToMeters(60));
        public Translation2d RED_PAD_1 = new Translation2d( Units.inchesToMeters(518), Units.inchesToMeters(108));
        public Translation2d RED_PAD_2 = new Translation2d( Units.inchesToMeters(518), Units.inchesToMeters(12));
      } 
        
}
