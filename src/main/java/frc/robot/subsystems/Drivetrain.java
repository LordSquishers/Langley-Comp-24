package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.swerve.TalonSwerve;

import java.util.Set;
import java.util.function.Supplier;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Swerve.PIDConstants.*;

public class Drivetrain extends TalonSwerve {
    private static Drivetrain instance;

    //TODO- all path following goes in here!
    public static Drivetrain getInstance() {
        if (instance == null) instance = new Drivetrain();
        return instance;
    }

    private Drivetrain() {
        super(
                new SwerveModuleConstants(FrontLeft.DRIVE_MOTOR_ID, FrontLeft.TURN_MOTOR_ID, FrontLeft.CANCODER_ID, Rotation2d.fromRadians(FrontLeft.ANGULAR_OFFSET)),
                new SwerveModuleConstants(FrontRight.DRIVE_MOTOR_ID, FrontRight.TURN_MOTOR_ID, FrontRight.CANCODER_ID, Rotation2d.fromRadians(FrontRight.ANGULAR_OFFSET)),
                new SwerveModuleConstants(BackLeft.DRIVE_MOTOR_ID, BackLeft.TURN_MOTOR_ID, BackLeft.CANCODER_ID, Rotation2d.fromRadians(BackLeft.ANGULAR_OFFSET)),
                new SwerveModuleConstants(BackRight.DRIVE_MOTOR_ID, BackRight.TURN_MOTOR_ID, BackRight.CANCODER_ID, Rotation2d.fromRadians(BackRight.ANGULAR_OFFSET))
        );

        AutoBuilder.configureHolonomic(this::getOdometryPose, this::setOdometryPose, this::getChassisSpeeds, this::relativeDrive,
                new HolonomicPathFollowerConfig(
                        PIDConstants.kPID_AUTO_TRANSLATION,
                        PIDConstants.kPID_AUTO_ROTATION,
                        MAX_VELOCITY_M_PER_S,
                        WHEEL_TO_WHEEL_DISTANCE_METERS,
                        new ReplanningConfig(true, true, PATH_REPLANNING_TOTAL_ERROR_THRESHOLD, PATH_REPLANNING_SPIKE_ERROR_THRESHOLD)
                ), () -> (DriverStation.getAlliance().get() == DriverStation.Alliance.Red), this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Command alignRobotToTargetCommand(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Pose2d> targetPose, boolean isAbsoluteDrive) {
        PIDController rotationPID = new PIDController(kP_ALIGN, kI_ALIGN, kD_ALIGN);
        rotationPID.setTolerance(ALIGN_ANGLE_TOLERANCE_DEG);
        rotationPID.enableContinuousInput(-180, 180);

        return new DeferredCommand(() ->
                // isFinished
                new FunctionalCommand(
                        () -> {
                        }, // Init
                        () -> { // Execute\
                            Translation2d currentPosition = getOdometryPose().getTranslation();
                            Translation2d vectorToTarget = currentPosition.minus(targetPose.get().getTranslation());
                            Rotation2d targetAngle = vectorToTarget.getAngle();

                            double rotationSpeed = 0.0;
                            rotationSpeed = rotationPID.calculate(getYaw().getDegrees(), targetAngle.getDegrees());
                            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed.get(), ySpeed.get(), rotationSpeed);
                            if (isAbsoluteDrive) {
                                absoluteDrive(chassisSpeeds, false);
                            } else {
                                relativeDrive(chassisSpeeds);
                            }
                        },
                        wasInterrupted -> { // End
                            rotationPID.close();
                            stopSwerveMotors();
                        },
                        rotationPID::atSetpoint, // isFinished
                        this
                ).repeatedly(),
                Set.of(this)
        );
    }

    public Command navigateToPosition(Supplier<Pose2d> targetPose) {
        return new DeferredCommand(() -> new FollowPathHolonomic(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(
                                new Pose2d(getOdometryPose().getTranslation(), new Rotation2d()),
                                targetPose.get()
                        ),
                        new PathConstraints(
                                MAX_VELOCITY_M_PER_S,
                                MAX_ACCELERATION_M_PER_S_2,
                                MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
                                MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_2
                        ),
                        new GoalEndState(0, targetPose.get().getRotation()),
                        false
                ),
                this::getOdometryPose,
                this::getChassisSpeeds,
                this::relativeDrive,
                new HolonomicPathFollowerConfig(
                        PIDConstants.kPID_AUTO_TRANSLATION,
                        PIDConstants.kPID_AUTO_ROTATION,
                        MAX_VELOCITY_M_PER_S,
                        WHEEL_TO_WHEEL_DISTANCE_METERS,
                        new ReplanningConfig(true, true, PATH_REPLANNING_TOTAL_ERROR_THRESHOLD, PATH_REPLANNING_SPIKE_ERROR_THRESHOLD)
                ),
                () -> (DriverStation.getAlliance().get() == DriverStation.Alliance.Red),
                this
        ),
                Set.of(this)
        );
    }

    public Command chasePoseRobotRelativeCommand(Supplier<Pose2d> target) {
        TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ALIGN_VELOCITY_PERCENT, MAX_ALIGN_ACCELERATION_PERCENT);
        ProfiledPIDController xController = new ProfiledPIDController(kP_NAVIGATE, kI_NAVIGATE, kD_NAVIGATE, X_CONSTRAINTS);
        PIDController omegaPID = new PIDController(kP_ALIGN, kI_ALIGN, kD_ALIGN);

        xController.setTolerance(NAVIGATE_POSITION_TOLERANCE_METERS);
        omegaPID.setTolerance(ALIGN_ANGLE_TOLERANCE_DEG);
        omegaPID.enableContinuousInput(-180, 180);

        return new DeferredCommand(() ->
                new FunctionalCommand(
                        () -> {
                        }, // Init
                        () -> { // Execute
                            double xSpeed = xController.calculate(0, target.get().getX());
                            double omegaSpeed = omegaPID.calculate(0, target.get().getRotation().getDegrees());

                            relativeDrive(new ChassisSpeeds(xSpeed, 0, omegaSpeed));
                        },
                        interrupted -> { // End
                            stopSwerveMotors();
                            omegaPID.close();
                            System.out.println("Robot is aligned with target pose!");
                        },
                        () -> (omegaPID.atSetpoint() && xController.atGoal()), // isFinished
                        this),
                Set.of(this));
    }

}
