package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Supplier;

import static frc.robot.Constants.Swerve.*;

public class TalonSwerve extends SubsystemBase {

    private SwerveDriveOdometry odometry;
    private TalonSwerveModule[] modules;
    private AHRS gyro;

    public TalonSwerve(Constants.Swerve.SwerveModuleConstants frontLeft, Constants.Swerve.SwerveModuleConstants frontRight, Constants.Swerve.SwerveModuleConstants backLeft, Constants.Swerve.SwerveModuleConstants backRight) {
        gyro = new AHRS(SPI.Port.kMXP);
        resetGyro();

        modules = new TalonSwerveModule[]{
                new TalonSwerveModule(0, frontLeft),
                new TalonSwerveModule(1, frontRight),
                new TalonSwerveModule(2, backLeft),
                new TalonSwerveModule(3, backRight)
        };
        odometry = new SwerveDriveOdometry(KINEMATICS, this.getYaw(), this.getModulePositions());

        var swerveDashboardTab = Shuffleboard.getTab("Swerve");
        swerveDashboardTab.addDouble("module 0 position", () -> getModulePositions()[0].distanceMeters);
        swerveDashboardTab.addDouble("module 1 position", () -> getModulePositions()[1].distanceMeters);
        swerveDashboardTab.addDouble("module 2 position", () -> getModulePositions()[2].distanceMeters);
        swerveDashboardTab.addDouble("module 3 position", () -> getModulePositions()[3].distanceMeters);
        for (TalonSwerveModule mod : modules) {
            swerveDashboardTab.addDouble(
                    "Mod " + mod.moduleID + " Cancoder", () -> mod.getAbsoluteRotation().getDegrees());
            swerveDashboardTab.addDouble(
                    "Mod " + mod.moduleID + " Integrated", () -> mod.getPosition().angle.getDegrees());
            swerveDashboardTab.addDouble(
                    "Mod " + mod.moduleID + " Velocity", () -> mod.getState().speedMetersPerSecond);
        }

        setName("Swerve");
        swerveDashboardTab.add(this);
    }

    @Override
    public void periodic() {
        // nice to reset periodically.
        if (DriverStation.isDisabled()) resetModulesToAbsolute();

        odometry.update(getYaw(), getModulePositions());
    }

    public void relativeDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_VELOCITY_M_PER_S);

        for (TalonSwerveModule module : modules) {
            module.setDesiredState(swerveModuleStates[module.moduleID], true);
        }
    }

    public void absoluteDrive(ChassisSpeeds speeds, boolean isOpenLoop) {
        var transformedSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getYaw());
        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(transformedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_VELOCITY_M_PER_S);

        for (TalonSwerveModule module : modules) {
            module.setDesiredState(swerveModuleStates[module.moduleID], isOpenLoop);
        }
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public void setOdometryPose(Pose2d pose) {
        odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_M_PER_S);

        for (TalonSwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleID], false);
        }
    }

    public Command manualRelativeDriveCommand(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed) {
        return new RunCommand(() -> this.relativeDrive(new ChassisSpeeds(xSpeed.get() * MAX_VELOCITY_M_PER_S * MAXIMUM_DRIVER_VELOCITY_PERCENT, ySpeed.get() * MAX_VELOCITY_M_PER_S * MAXIMUM_DRIVER_VELOCITY_PERCENT, rotationSpeed.get() * MAX_ANGULAR_VELOCITY_RAD_PER_SEC * MAXIMUM_DRIVER_OMEGA_PERCENT)), this).repeatedly();
    }

    public Command manualAbsoluteDriveCommand(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed) {
        return new RunCommand(() -> this.absoluteDrive(new ChassisSpeeds(xSpeed.get() * MAX_VELOCITY_M_PER_S * MAXIMUM_DRIVER_VELOCITY_PERCENT, ySpeed.get() * MAX_VELOCITY_M_PER_S * MAXIMUM_DRIVER_VELOCITY_PERCENT, rotationSpeed.get() * MAX_ANGULAR_VELOCITY_RAD_PER_SEC * MAXIMUM_DRIVER_OMEGA_PERCENT), true), this).repeatedly();
    }

    public Rotation2d getYaw() { //TODO- check w/ 611 code
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    private SwerveModulePosition[] getModulePositions() {
        var positions = new SwerveModulePosition[4];
        for (TalonSwerveModule module : modules) {
            positions[module.moduleID] = module.getPosition();
        }
        return positions;
    }

    public void stopSwerveMotors() {
        absoluteDrive(new ChassisSpeeds(0, 0, 0), true);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (TalonSwerveModule module : modules) {
            states[module.moduleID] = module.getState();
        }

        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public void resetModulesToAbsolute() {
        for (TalonSwerveModule module : modules) {
            module.syncAngleToAbsolute();
        }
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void setGyroYaw(double degrees) {
        gyro.setAngleAdjustment(degrees);
    }

}
