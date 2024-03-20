package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import java.util.function.Supplier;

import static frc.robot.Constants.Pivot.FFConstants.*;
import static frc.robot.Constants.Pivot.PIDConstants.*;
import static frc.robot.Constants.Pivot.*;

public class Pivot extends ProfiledPIDSubsystem {

    public enum State {
        INTAKE, STOW, AUTOMATIC, AMP, SUBWOOFER, PODIUM
    }
    private State state = State.STOW;

    public enum Status {
        READY, ADJUSTING;
    }
    private Status status = Status.READY;

    private static Pivot instance;

    /* Initialize Motor Controllers */
    private final CANSparkMax leftPivotMotor = new CANSparkMax(LEFT_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightPivotMotor = new CANSparkMax(RIGHT_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);

    /* Initialize Absolute Encoder */
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ABS_ENCODER_RIO_CHANNEL);

    /* System Controllers */
    private final ArmFeedforward feedForward = new ArmFeedforward(kS_VOLTS, kG_VOLTS, kV_VOLT_SEC_PER_DEG, kA_VOLT_SEC_2_PER_DEG);

    /**
     * Returns the instance of the Pivot.
     * If the instance doesn't exist, creates a new one.
     * @return the instance of the pivot.
     */
    public static Pivot getInstance() {
        if (instance == null) return new Pivot();
        else return instance;
    }

    private Pivot() {
        super(new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC, MAX_ACCEL_DEG_PER_SEC_2)));

        absoluteEncoder.setDistancePerRotation(360);
        setGoal(ANGLE_STOW_DEG);
        getController().setTolerance(GOAL_ANGLE_TOLERANCE_DEG);
    }

    /**
     * Continuously called by the scheduler to update the system data.
     */
    @Override
    public void periodic() {
        super.periodic();

        updateStatus();

        /* State Decisions */
        switch (state) {
            case INTAKE:
                setPosition(ANGLE_INTAKE_DEG);
            case STOW:
                setPosition(ANGLE_STOW_DEG);
            case AUTOMATIC:
                setPosition(calculateAutomaticAngle());
            case AMP:
                setPosition(ANGLE_AMP_DEG);
            case SUBWOOFER:
                setPosition(ANGLE_SUBWOOFER_DEG);
            case PODIUM:
                setPosition(ANGLE_PODIUM_DEG);
        }

        postData();
    }

    /**
     * Sets the motor voltage based on output and states.
     * @param v The output voltage value.
     * @param state The current TrapezoidProfile state.
     */
    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        double feedforward = feedForward.calculate(state.position, state.velocity);
        setMotorVoltage(v + feedforward);
    }

    /**
     * Returns the raw value from the encoder.
     * @return raw value from the encoder.
     */
    @Override
    protected double getMeasurement() {
        return getPosition();
    }

    private double calculateAutomaticAngle() {
        return 0; //TODO- Implement
    }

    /**
     * Creates an InstantCommand using a specified State.
     * This command will set the state of the pivot.
     * @param state The State to be set.
     * @return An InstantCommand that sets the state of the pivot.
     */
    public InstantCommand setStateCommand(State state) {
        return new InstantCommand(() -> this.setState(state));
    }

    private void setState(State state) {
        this.state = state;
    }

    private void updateStatus() {
        if(isAtGoal()) this.status = Status.READY;
        else this.status = Status.ADJUSTING;
    }

    /**
     * Returns the current state of the pivot.
     * @return the current state of the pivot.
     */
    public Supplier<State> getState() {
        return () -> this.state;
    }

    /**
     * Returns the current status of the pivot.
     * @return the current status of the pivot.
     */
    public Supplier<Status> getStatus() {
        return () -> this.status;
    }

    private void setMotorVoltage(double volts) {
        leftPivotMotor.setVoltage(volts);
        rightPivotMotor.setVoltage(volts);
    }

    /**
     * Returns the Position (Angle) from the encoder.
     * @return the Position (Angle) from the encoder.
     */
    public double getPosition() {
        return absoluteEncoder.getDistance() + ABS_ENCODER_OFFSET;
    }

    /**
     * Sets the goal position (desired angle) for the pivot.
     * @param position The desired angle for the pivot.
     */
    public void setPosition(double position) {
        setGoal(position);
    }

    /**
     * Checks if the current position matches the goal position.
     * @return True if the current position matches the goal, false otherwise.
     */
    public boolean isAtGoal() {
        return getController().atGoal();
    }

    private void postData() {
        SmartDashboard.putNumber("Pivot Goal (deg): ", this.getController().getGoal().position);

        SmartDashboard.putNumber("Pivot Encoder (deg): ", getPosition());
        SmartDashboard.putBoolean("Pivot Encoder Connected? ", absoluteEncoder.isConnected());
        SmartDashboard.putNumber("Pivot Right Voltage: ", leftPivotMotor.getBusVoltage());
        SmartDashboard.putNumber("Pivot Left Voltage: ", rightPivotMotor.getBusVoltage());

        SmartDashboard.putString("Pivot State: ", state.toString());
        SmartDashboard.putString("Pivot Status: ", status.toString());
    }
}
