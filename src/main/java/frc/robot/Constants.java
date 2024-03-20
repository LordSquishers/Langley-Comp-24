// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static frc.robot.Constants.Maths.INCHES_TO_METERS;
import static frc.robot.Constants.SubsystemType.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Maths {
        public static final double DEG_TO_RAD = 2 * Math.PI / 180;
        public static final double INCHES_TO_METERS = 0.0254;
    }

    public static final SubsystemType[] ENABLED_SUBSYSTEMS = { PIVOT };

    public enum SubsystemType {
        SWERVE_DRIVETRAIN, PIVOT, SHOOTER, INTAKE
    }

    public static boolean isSubsystemAvailable(SubsystemType type) {
        for (var subsystemType : ENABLED_SUBSYSTEMS) {
            if (subsystemType == type) return true;
        }
        return false;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR_CAN_ID = 4;
        public static final int FEED_MOTOR_CAN_ID = 5;

        public static final int BEAM_BREAK_DIO_CHANNEL = 1;

        public static final double INTAKE_VOLT_PERCENTAGE = 0.6;
        public static final double OUTTAKE_VOLT_PERCENTAGE = -0.4;
        public static final double FEED_VOLT_PERCENTAGE = 0.6;
        public static final double OFF_VOLT_PERCENTAGE = 0.0; // for completion lol
    }

    public static final class Shooter { // SHOOTER RUNS IN RPM
        public static final int MOTOR_CAN_ID = 3;

        public static final boolean ENABLE_IDLE_WHEN_NOTE_PRESENT = true;

        public static final double AMP_RPM = 1000;
        public static final double SUBWOOFER_RPM = 3500;
        public static final double PODIUM_RPM = 4250;
        public static final double IDLE_RPM = 2000;

        public static final double MOTOR_TO_WHEEL_GEAR_RATIO = 25.0 / 2.0;
        public static final int ALT_ENCODER_COUNTS_PER_REV = 4096;
        public static final double RPM_TOLERANCE = 10;

        public static final class PIDConstants {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class Pivot { // PIVOT RUNS IN DEGREES //
        public static final int LEFT_MOTOR_CAN_ID = 1, RIGHT_MOTOR_CAN_ID = 2;
        public static final int ABS_ENCODER_RIO_CHANNEL = 0;

        public static final double MAX_VELOCITY_DEG_PER_SEC = 0;
        public static final double MAX_ACCEL_DEG_PER_SEC_2 = 0;

        public static final double ABS_ENCODER_OFFSET = 0;
        public static final double GOAL_ANGLE_TOLERANCE_DEG = 0.1;

        public static final double ANGLE_STOW_DEG = 20;
        public static final double ANGLE_INTAKE_DEG = 0;
        public static final double ANGLE_AMP_DEG = 100;
        public static final double ANGLE_SUBWOOFER_DEG = 60;
        public static final double ANGLE_PODIUM_DEG = 30;

        public static final class PIDConstants {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static final class FFConstants {
            public static final double kS_VOLTS = 0;
            public static final double kG_VOLTS = 0;
            public static final double kV_VOLT_SEC_PER_DEG = 0;
            public static final double kA_VOLT_SEC_2_PER_DEG = 0;
        }
    }

    public static final class OperatorInput {
        public static final int DRIVER_CONTROLLER_ID = 0;
        public static final int OPERATOR_CONTROLLER_ID = 1;
    }

    public record FiringSetpoint(double shooterRPM, double pivotAngle) {
        public static final FiringSetpoint AMP = new FiringSetpoint(Shooter.AMP_RPM, Pivot.ANGLE_AMP_DEG);
        public static final FiringSetpoint SUBWOOFER = new FiringSetpoint(Shooter.SUBWOOFER_RPM, Pivot.ANGLE_SUBWOOFER_DEG);
        public static final FiringSetpoint PODIUM = new FiringSetpoint(Shooter.PODIUM_RPM, Pivot.ANGLE_PODIUM_DEG);
    }

    public static final class Swerve {
        public static final double CURRENT_LIMIT_TURN_MOTOR_AMPS = 40;
        public static final double CURRENT_LIMIT_DRIVE_MOTOR_AMPS = 40;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double WHEEL_DIAMETER_INCHES = 4.0;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_INCHES * INCHES_TO_METERS * Math.PI;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
        public static final double DRIVE_GEAR_RATIO = 8.14;
        public static final double CANCODER_COUNTS_PER_REV = 2048.0;

        public static final double WHEEL_TO_WHEEL_DISTANCE_INCHES = 27.0;
        public static final double WHEEL_TO_WHEEL_DISTANCE_METERS = WHEEL_TO_WHEEL_DISTANCE_INCHES * INCHES_TO_METERS;

        public static final double MAX_VELOCITY_M_PER_S = 5.4864;
        public static final double MAX_ACCELERATION_M_PER_S_2 = 3.0;
        public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 10.0;

        public static final double MAXIMUM_DRIVER_VELOCITY_PERCENT = 0.6;
        public static final double MAXIMUM_DRIVER_OMEGA_PERCENT = 0.6;

        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(WHEEL_TO_WHEEL_DISTANCE_METERS / 2.0, WHEEL_TO_WHEEL_DISTANCE_METERS / 2.0),
                        new Translation2d(WHEEL_TO_WHEEL_DISTANCE_METERS / 2.0, -WHEEL_TO_WHEEL_DISTANCE_METERS / 2.0),
                        new Translation2d(-WHEEL_TO_WHEEL_DISTANCE_METERS / 2.0, WHEEL_TO_WHEEL_DISTANCE_METERS / 2.0),
                        new Translation2d(-WHEEL_TO_WHEEL_DISTANCE_METERS / 2.0, -WHEEL_TO_WHEEL_DISTANCE_METERS / 2.0)
                );

        public static final class PIDConstants {
            public static final double kP_TURN_MOTOR = 0.3;
            public static final double kI_TURN_MOTOR = 0.0;
            public static final double kD_TURN_MOTOR = 0.0;

            public static final double kP_DRIVE_MOTOR = 0.3;
            public static final double kI_DRIVE_MOTOR = 0.1;
            public static final double kD_DRIVE_MOTOR = 0.02;

            // Division by 12 for volts to % output.
            public static final double kS_DRIVE_MOTOR = 0.32 / 12.0;
            public static final double kV_DRIVE_MOTOR = 1.51 / 12.0;
            public static final double kA_DRIVE_MOTOR = 0.27 / 12.0;
        }

        public static final class FrontLeft { // Module 0
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int TURN_MOTOR_ID = 2;
            public static final int CANCODER_ID = 10;
            public static final double ANGULAR_OFFSET = -Math.PI / 2.0;
        }

        public static final class FrontRight { // Module 1
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int TURN_MOTOR_ID = 4;
            public static final int CANCODER_ID = 11;
            public static final double ANGULAR_OFFSET = 0.0;
        }

        public static final class BackLeft { // Module 2
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int TURN_MOTOR_ID = 6;
            public static final int CANCODER_ID = 12;
            public static final double ANGULAR_OFFSET = Math.PI;
        }

        public static final class BackRight { // Module 3
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int TURN_MOTOR_ID = 8;
            public static final int CANCODER_ID = 13;
            public static final double ANGULAR_OFFSET = Math.PI / 2.0;
        }

        public static final class SwerveModuleConstants {
            public final int driveID, turnID, encoderID;
            public final Rotation2d rotationOffset;

            public SwerveModuleConstants(int driveID, int turnID, int encoderID, Rotation2d rotationOffset) {
                this.driveID = driveID;
                this.turnID = turnID;
                this.encoderID = encoderID;
                this.rotationOffset = rotationOffset;
            }
        }
    }

}
