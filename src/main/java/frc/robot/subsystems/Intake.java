package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {

    //TODO- implement beam break
    private final DigitalInput beambreak = new DigitalInput(BEAM_BREAK_DIO_CHANNEL);

    public enum State {
        OFF, INTAKE, OUTTAKE, FEED
    }
    private State state = State.OFF;

    public enum Status {
        EMPTY, HAS_NOTE;
    }
    private Status status = Status.EMPTY;

    private static Intake instance;

    /* Initialize Motor Controller */
    private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax feedMotor = new CANSparkMax(FEED_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    private Intake() {
    }

    @Override
    public void periodic() {
        updateStatus();
        updateState();

        /* State Decisions */
        switch (state) {
            case OFF: // no movement.
                setIntakeMotorVoltage(OFF_VOLT_PERCENTAGE);
                setFeedMotorVoltage(OFF_VOLT_PERCENTAGE);
            case INTAKE: // intake, but not feeder(?)
                setIntakeMotorVoltage(INTAKE_VOLT_PERCENTAGE);
                setFeedMotorVoltage(INTAKE_VOLT_PERCENTAGE); //TODO - maybe don't want this?
            case OUTTAKE: // clear note by fully outtaking.
                setIntakeMotorVoltage(OUTTAKE_VOLT_PERCENTAGE);
                setFeedMotorVoltage(OUTTAKE_VOLT_PERCENTAGE);
            case FEED: // feed note into shooter!
                setIntakeMotorVoltage(FEED_VOLT_PERCENTAGE);
                setFeedMotorVoltage(FEED_VOLT_PERCENTAGE);
        }

        postData();
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
        if(hasNote()) this.status = Status.HAS_NOTE;
        else this.status = Status.EMPTY;
    }

    private void updateState() {
        // only disable intaking when there is a note present.
        if(status == Status.HAS_NOTE && state == State.INTAKE) state = State.OFF;
        // disable once note has left intake.
        //TODO- consider adding second delay to ensure note has left intake/feeder wheels depending on BB.
        if(status == Status.EMPTY && state == State.FEED) state = State.OFF;
    }

    private boolean hasNote() {
         return beambreak.get();
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

    private void setIntakeMotorVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    private void setFeedMotorVoltage(double volts) {
        feedMotor.setVoltage(volts);
    }


    private void postData() {
        SmartDashboard.putNumber("Intake Motor Voltage", intakeMotor.getBusVoltage());
        SmartDashboard.putNumber("Feed Motor Voltage", feedMotor.getBusVoltage());
        SmartDashboard.putString("Intake State: ", state.toString());
        SmartDashboard.putString("Intake Status: ", status.toString());
        SmartDashboard.putBoolean("Note in Intake: ", beambreak.get());
    }
}
