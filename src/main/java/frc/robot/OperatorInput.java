package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.*;

import static frc.robot.Constants.OperatorInput.*;

public class OperatorInput {

    private static OperatorInput instance = null;

    /* Controller Definitions */
    private XboxController driver = new XboxController(DRIVER_CONTROLLER_ID);
    private XboxController operator = new XboxController(OPERATOR_CONTROLLER_ID);

    private OperatorInput() {
        // Private constructor to prevent instantiation
    }

    public static OperatorInput getInstance() {
        if (instance == null) {
            instance = new OperatorInput();
        }
        return instance;
    }

    public void configureButtonBindings() {
        /* Climber Commands */
        if (Constants.isSubsystemAvailable(Constants.SubsystemType.CLIMBER)) configureClimberButtonBindings();

        /* Pivot Commands */
        if (Constants.isSubsystemAvailable(Constants.SubsystemType.PIVOT)) configurePivotButtonBindings();

        /* Shooter Commands */
        if (Constants.isSubsystemAvailable(Constants.SubsystemType.SHOOTER)) configureShooterButtonBindings();

        /* Intake Commands */
        if (Constants.isSubsystemAvailable(Constants.SubsystemType.INTAKE)) configureIntakeButtonBindings();

        /* Driver Commands */
        if (Constants.isSubsystemAvailable(Constants.SubsystemType.SWERVE_DRIVETRAIN)) configureDrivetrainButtonBindings();
    }

    private void configureDrivetrainButtonBindings() {
        var drivetrain = Drivetrain.getInstance();

        drivetrain.setDefaultCommand(
                drivetrain.manualAbsoluteDriveCommand(driver::getLeftX, driver::getLeftY, driver::getRightX)
        );

        Xbox.RB_BUTTON(driver).whileTrue(
                drivetrain.alignRobotToTargetCommand(driver::getLeftX, driver::getLeftY, () -> {
                    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                        return Coordinates.RED_SPEAKER;
                    } else {
                        return Coordinates.BLUE_SPEAKER;
                    }
                }, true)
        );

        // Xbox.A_BUTTON(driver).whileTrue(drivetrain.chasePoseRobotRelativeCommand(notePosition));
    }

    private void configureClimberButtonBindings() {
        var climber = Climber.getInstance();

        // lower / raise climber.
        Xbox.LB_BUTTON(operator).onTrue(climber.setStateCommand(Climber.State.STOW));
        Xbox.RB_BUTTON(operator).onTrue(climber.setStateCommand(Climber.State.EXTEND));
    }

    private void configureShooterButtonBindings() {
        var shooter = Shooter.getInstance();

        // Amp Scoring (will not output note, must be fed).
        Xbox.DPAD_UP(operator).onTrue(shooter.setStateCommand(Shooter.State.AMP));
        // Automatic Scoring (will not output note, must be fed).
        Xbox.RIGHT_STICK_CLICK_BUTTON(operator).onTrue(shooter.setStateCommand(Shooter.State.AUTOMATIC));
        // Turn off shooter
        Xbox.X_BUTTON(operator).onTrue(shooter.setStateCommand(Shooter.State.OFF));
    }

    private void configurePivotButtonBindings() {
        var pivot = Pivot.getInstance();

        // Amp scoring (will not output note, must be fed).
        Xbox.DPAD_UP(operator).onTrue(pivot.setStateCommand(Pivot.State.AMP));
        // Automatic scoring (will not output note, must be fed).
        Xbox.RIGHT_STICK_CLICK_BUTTON(operator).onTrue(pivot.setStateCommand(Pivot.State.AUTOMATIC));
        // Pivot down / Intake notes
        Xbox.DPAD_DOWN(operator).onTrue(pivot.setStateCommand(Pivot.State.INTAKE));
        Xbox.A_BUTTON(operator).onTrue(pivot.setStateCommand(Pivot.State.INTAKE));
    }

    private void configureIntakeButtonBindings() {
        var intake = Intake.getInstance();

        // Intake / Outtake
        Xbox.A_BUTTON(operator).onTrue(intake.setStateCommand(Intake.State.INTAKE));
        Xbox.Y_BUTTON(operator).onTrue(intake.setStateCommand(Intake.State.OUTTAKE));
        // Feed note to score.
        Xbox.B_BUTTON(operator).onTrue(intake.setStateCommand(Intake.State.FEED));
    }

    public static final class Xbox {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;

        public static final int SELECT = 7;
        public static final int MENU = 8;

        public static final int LEFT_STICK_CLICK = 9;
        public static final int RIGHT_STICK_CLICK = 10;

        public static final int UP_ARR = 0;
        public static final int RIGHT_ARR = 90;
        public static final int DOWN_ARR = 180;
        public static final int LEFT_ARR = 270;

        public static JoystickButton A_BUTTON(XboxController controller) { return new JoystickButton(controller, A); }
        public static JoystickButton B_BUTTON(XboxController controller) { return new JoystickButton(controller, B); }
        public static JoystickButton X_BUTTON(XboxController controller) { return new JoystickButton(controller, X); }
        public static JoystickButton Y_BUTTON(XboxController controller) { return new JoystickButton(controller, Y); }
        public static JoystickButton LB_BUTTON(XboxController controller) { return new JoystickButton(controller, LB); }
        public static JoystickButton RB_BUTTON(XboxController controller) { return new JoystickButton(controller, RB); }
        public static JoystickButton SELECT_BUTTON(XboxController controller) { return new JoystickButton(controller, SELECT); }
        public static JoystickButton MENU_BUTTON(XboxController controller) { return new JoystickButton(controller, MENU); }
        public static JoystickButton LEFT_STICK_CLICK_BUTTON(XboxController controller) { return new JoystickButton(controller, LEFT_STICK_CLICK); }
        public static JoystickButton RIGHT_STICK_CLICK_BUTTON(XboxController controller) { return new JoystickButton(controller, RIGHT_STICK_CLICK); }
        public static POVButton DPAD_UP(XboxController controller) { return new POVButton(controller, UP_ARR); }
        public static POVButton DPAD_RIGHT(XboxController controller) { return new POVButton(controller, RIGHT_ARR); }
        public static POVButton DPAD_DOWN(XboxController controller) { return new POVButton(controller, DOWN_ARR); }
        public static POVButton DPAD_LEFT(XboxController controller) { return new POVButton(controller, LEFT_ARR); }

    }

}