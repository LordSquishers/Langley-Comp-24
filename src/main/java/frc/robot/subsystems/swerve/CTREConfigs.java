package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static frc.robot.Constants.Swerve.PIDConstants.*;
import static frc.robot.Constants.Swerve.*;

public class CTREConfigs {

    public TalonFXConfiguration driveMotorConfig, turnMotorConfig;
    public CANcoderConfiguration absoluteEncoderConfig;

    public CTREConfigs() {
        /* Turn Motor Configs */
        turnMotorConfig = new TalonFXConfiguration().withSlot0(
                new Slot0Configs().withKP(kP_TURN_MOTOR).withKI(kI_TURN_MOTOR).withKD(kD_TURN_MOTOR)
        ).withCurrentLimits(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT_TURN_MOTOR_AMPS)
        );

        /* Drive Motor Configs */
        driveMotorConfig = new TalonFXConfiguration().withSlot0(
                new Slot0Configs().withKP(kP_DRIVE_MOTOR).withKI(kI_DRIVE_MOTOR).withKD(kD_DRIVE_MOTOR)
        ).withCurrentLimits(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT_DRIVE_MOTOR_AMPS)
        ).withOpenLoopRamps(
                new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(OPEN_LOOP_RAMP)
        ).withClosedLoopRamps(
                new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(CLOSED_LOOP_RAMP)
        );

        /* CANcoder Config */
        absoluteEncoderConfig = new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs()
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        );
    }
}
