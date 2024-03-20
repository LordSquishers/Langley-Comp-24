package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

import java.util.function.Supplier;

import static frc.robot.Constants.Shooter.PIDConstants.*;
import static frc.robot.Constants.Shooter.*;

public class Shooter extends PIDSubsystem {

    public enum State {
        OFF, AUTOMATIC, IDLE, SUBWOOFER, PODIUM, AMP
    }
    private State state = State.OFF;

    public enum Status {
        READY, ADJUSTING;
    }
    private Status status = Status.READY;

    private static Shooter instance;

    /* Initialize Motor Controller and Encoder */
    private final CANSparkMax shooterMotor = new CANSparkMax(MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder encoder;

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }

    private Shooter() {
        super(new PIDController(kP, kI, kD), 0);

        getController().setTolerance(RPM_TOLERANCE);
        setSetpoint(0);

        encoder = shooterMotor.getAlternateEncoder(ALT_ENCODER_COUNTS_PER_REV);
        encoder.setVelocityConversionFactor(MOTOR_TO_WHEEL_GEAR_RATIO);
    }

    @Override
    public void periodic() {
        super.periodic();

        updateState();
        updateStatus();

        /* State Decisions */
        switch (state) {
            case IDLE:
                setVelocity(IDLE_RPM);
            case AUTOMATIC:
                setVelocity(calculateAutomaticVelocity());
            case AMP:
                setVelocity(AMP_RPM);
            case SUBWOOFER:
                setVelocity(SUBWOOFER_RPM);
            case PODIUM:
                setVelocity(PODIUM_RPM);
        }

        postData();
    }

    private double calculateAutomaticVelocity() {
        return 0; // TODO- implement
    }

    @Override
    protected void useOutput(double voltage, double setpoint) {
        setMotorVoltage(voltage);
    }

    @Override
    protected double getMeasurement() {
        return encoder.getVelocity();
    }

    /**
     * Creates an InstantCommand using a specified State.
     * This command will set the state of the shooter.
     * @param state The State to be set.
     * @return An InstantCommand that sets the state of the shooter.
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

    private void updateState() {
        // If the intake is real and we have a note, bring the shooter up to idle only if it was off.
        if (ENABLE_IDLE_WHEN_NOTE_PRESENT && Constants.isSubsystemAvailable(Constants.SubsystemType.INTAKE)) {
            if (Intake.getInstance().getStatus().get() == Intake.Status.HAS_NOTE && state == State.OFF) setState(State.IDLE);
        }
    }

    /**
     * Returns the current state of the shooter.
     * @return the current state of the shooter.
     */
    public Supplier<State> getState() {
        return () -> this.state;
    }

    /**
     * Returns the current status of the shooter.
     * @return the current status of the shooter.
     */
    public Supplier<Status> getStatus() {
        return () -> this.status;
    }

    private void setMotorVoltage(double volts) {
        shooterMotor.setVoltage(volts);
    }

    /**
     * Returns the Velocity (RPM) from the encoder.
     * @return the Velocity (RPM) from the encoder.
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Sets the goal velocity (desired RPM) for the shooter.
     * @param velocity The desired RPM for the shooter.
     */
    public void setVelocity(double velocity) {
        getController().setSetpoint(velocity);
    }

    /**
     * Checks if the current velocity matches the goal velocity.
     * @return True if the current velocity matches the goal, false otherwise.
     */
    public boolean isAtGoal() {
        return getController().atSetpoint();
    }

    private void postData() {
        SmartDashboard.putNumber("Shooter Goal (RPM): ", this.getController().getSetpoint());

        SmartDashboard.putNumber("Shooter Encoder (RPM): ", encoder.getVelocity());
        SmartDashboard.putNumber("Shooter Motor Voltage", shooterMotor.getBusVoltage());
        SmartDashboard.putString("Shooter State: ", state.toString());
        SmartDashboard.putString("Shooter Status: ", status.toString());
    }
}
