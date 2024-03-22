package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.Climber.PIDConstants.*;

public class Climber extends ProfiledPIDSubsystem {

    private static Climber instance;

    public static Climber getInstance() {
        if(instance == null) instance = new Climber();
        return instance;
    }

    private State state;
    private Status status;

    public enum State {
        STOW, EXTEND
    }

    public enum Status {
        FREE, AT_LIMIT
    }

    private final CANSparkMax leftClimbMotor = new CANSparkMax(LEFT_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightClimbMotor = new CANSparkMax(RIGHT_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);

    private final DigitalInput leftLimitSwitch = new DigitalInput(LEFT_LIMIT_SWITCH_DIO);
    private final DigitalInput rightLimitSwitch = new DigitalInput(RIGHT_LIMIT_SWITCH_DIO);

    private Climber() {
        super(new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC, MAX_ACCEL_DEG_PER_SEC_2)));

        getController().setTolerance(GOAL_ANGLE_TOLERANCE_DEG);

        // Set motors to brake mode
        leftClimbMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightClimbMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        // Have right motor follow left. Disable if you want independent hooks.
        rightClimbMotor.follow(leftClimbMotor, true);

        // Save configuration
        leftClimbMotor.burnFlash();
        rightClimbMotor.burnFlash();
    }

    @Override
    public void periodic() {
        super.periodic();

        switch (state) {
            case STOW -> setClimberPosition(MINIMUM_CLIMBER_POSITION_DEG);
            case EXTEND -> setClimberPosition(MAXIMUM_CLIMBER_POSITION_DEG);
        }

        updateEncoderValues();
        updateStatus();

        postData();
    }

    private void updateEncoderValues() {
        if (leftLimitSwitch.get()) leftClimbMotor.getEncoder().setPosition(MINIMUM_CLIMBER_POSITION_DEG);
        if (rightLimitSwitch.get()) rightClimbMotor.getEncoder().setPosition(MINIMUM_CLIMBER_POSITION_DEG);
    }

    private void updateStatus() {
        if (leftLimitSwitch.get() && rightLimitSwitch.get()) status = Status.AT_LIMIT;
        else status = Status.FREE;
    }

    /**
     * Creates an InstantCommand using a specified State.
     * This command will set the state of the climber.
     * @param state The State to be set.
     * @return An InstantCommand that sets the state of the climber.
     */
    public InstantCommand setStateCommand(State state) {
        return new InstantCommand(() -> this.setState(state));
    }

    private void setState(State state) {
        this.state = state;
    }

    private void setClimberPosition(double positionInDegrees) {
        setGoal(positionInDegrees);
    }

    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        setClimberMotorVoltage(v);
    }

    @Override
    protected double getMeasurement() {
        return (leftClimbMotor.getEncoder().getPosition() + rightClimbMotor.getEncoder().getPosition()) / 2.0;
    }

    private void setClimberMotorVoltage(double voltage) {
        leftClimbMotor.setVoltage(voltage);
        rightClimbMotor.setVoltage(voltage);
    }

    /**
     * Checks if the current position matches the goal position.
     * @return True if the current position matches the goal, false otherwise.
     */
    public boolean isAtGoal() {
        return getController().atGoal();
    }
    
    /**
     * Returns the current state of the climber.
     * @return the current state of the climber.
     */
    public Supplier<State> getState() {
        return () -> this.state;
    }

    /**
     * Returns the current status of the climber.
     * @return the current status of the climber.
     */
    public Supplier<Status> getStatus() {
        return () -> this.status;
    }

    private void postData() {
        SmartDashboard.putNumber("Climber Goal (deg):", getController().getGoal().position);
        SmartDashboard.putNumber("Left Climber Pos (deg):", leftClimbMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Climber Pos (deg):", rightClimbMotor.getEncoder().getPosition());

        SmartDashboard.putString("Climber State: ", state.toString());
        SmartDashboard.putString("Climber Status: ", status.toString());
    }

}
