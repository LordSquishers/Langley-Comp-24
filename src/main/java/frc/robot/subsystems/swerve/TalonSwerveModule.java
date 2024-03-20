package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

import static frc.robot.Constants.Swerve.PIDConstants.*;
import static frc.robot.Constants.Swerve.*;

public class TalonSwerveModule {

    public int moduleID;
    private Rotation2d rotationOffset, lastAngle;

    private TalonFX driveMotor, turnMotor;
    private CANcoder absoluteEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS_DRIVE_MOTOR, kV_DRIVE_MOTOR, kA_DRIVE_MOTOR);

    public TalonSwerveModule(int moduleID, Constants.Swerve.SwerveModuleConstants moduleConstants) {
        this.moduleID = moduleID;
        this.rotationOffset = moduleConstants.rotationOffset;

        CTREConfigs ctreConfigs = new CTREConfigs();

        absoluteEncoder = new CANcoder(moduleConstants.encoderID);
        configAbsoluteEncoder(ctreConfigs);

        turnMotor = new TalonFX(moduleConstants.turnID);
        configTurnMotor(ctreConfigs);

        driveMotor = new TalonFX(moduleConstants.driveID);
        configDriveMotor(ctreConfigs);

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public SwerveModuleState getState() {
        var driveMotorRPM = turnMotor.getPosition().getValue() * 600.0 / CANCODER_COUNTS_PER_REV;
        var wheelRPM = driveMotorRPM / DRIVE_GEAR_RATIO;
        var wheelMPS = wheelRPM * WHEEL_CIRCUMFERENCE_METERS / 60.0; // why 60 and 600? no clue
        return new SwerveModuleState(
                wheelMPS, getAngle()
        );
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveMotor.set(desiredState.speedMetersPerSecond / MAX_VELOCITY_M_PER_S);
        } else {
            double wheelRPM = desiredState.speedMetersPerSecond * 60.0 / WHEEL_CIRCUMFERENCE_METERS;
            double velocity = wheelRPM * DRIVE_GEAR_RATIO * CANCODER_COUNTS_PER_REV / 600.0;
            driveMotor.setControl(new VelocityVoltage(velocity));
        }
    }
    public SwerveModulePosition getPosition() {
        var meters = driveMotor.getPosition().getValue() * WHEEL_CIRCUMFERENCE_METERS / (DRIVE_GEAR_RATIO * CANCODER_COUNTS_PER_REV);
        return new SwerveModulePosition(
                meters, getAngle()
        );
    }

    public Rotation2d getAngle() {
        var degrees = turnMotor.getPosition().getValue() * 360.0 / (TURN_GEAR_RATIO * CANCODER_COUNTS_PER_REV);
        return Rotation2d.fromDegrees(degrees);
    }

    private void setAngle(SwerveModuleState desiredState) {
        // prevent jittering if less than 5cm/s.
        Rotation2d desiredAngle = (Math.abs(desiredState.speedMetersPerSecond) <= MAX_VELOCITY_M_PER_S * 0.01) ? lastAngle : desiredState.angle;
        turnMotor.setControl(new PositionVoltage(desiredAngle.getDegrees() * TURN_GEAR_RATIO * CANCODER_COUNTS_PER_REV / 360.0));
        lastAngle = desiredAngle;
    }

    public void syncAngleToAbsolute() {
        double absolutePosition = (getAbsoluteRotation().getDegrees() - rotationOffset.getDegrees()) * TURN_GEAR_RATIO * CANCODER_COUNTS_PER_REV / 360.0;
        turnMotor.setPosition(absolutePosition);
    }

    public Rotation2d getAbsoluteRotation() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition().getValue());
    }

    private void configDriveMotor(CTREConfigs ctreConfigs) {
        driveMotor.getConfigurator().DefaultTimeoutSeconds = 50;
        driveMotor.getConfigurator().apply(ctreConfigs.driveMotorConfig);
        driveMotor.setInverted(false); // not in constants because should never change
        driveMotor.setNeutralMode(NeutralModeValue.Brake); // same deal here
        driveMotor.setPosition(0); // reset motor encoder to 0.
    }

    private void configTurnMotor(CTREConfigs ctreConfigs) {
        turnMotor.getConfigurator().DefaultTimeoutSeconds = 50;
        turnMotor.getConfigurator().apply(ctreConfigs.turnMotorConfig);
        turnMotor.setInverted(false); // not in constants because should never change
        turnMotor.setNeutralMode(NeutralModeValue.Brake); // same deal here
        turnMotor.setPosition(0); // reset motor encoder to 0.
    }

    private void configAbsoluteEncoder(CTREConfigs ctreConfigs) {
        absoluteEncoder.getConfigurator().DefaultTimeoutSeconds = 50;
        absoluteEncoder.getConfigurator().apply(ctreConfigs.absoluteEncoderConfig);
    }
}
