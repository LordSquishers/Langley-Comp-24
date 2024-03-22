// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //TODO - implement vision!

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    /* Subsystem Initialization */
    for (Constants.SubsystemType subsystemType : Constants.ENABLED_SUBSYSTEMS) {
      switch (subsystemType) {
        case PIVOT -> Pivot.getInstance();
        case INTAKE -> Intake.getInstance();
        case SHOOTER -> Shooter.getInstance();
        case SWERVE_DRIVETRAIN -> Drivetrain.getInstance();
        case CLIMBER -> Climber.getInstance();
        default -> throw new IllegalStateException("Subsystem does not exist: " + subsystemType);
      }
    }

    OperatorInput.getInstance().configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return the autonomous command given by the drop-down selector in ShuffleBoard
    return autoChooser.getSelected();
  }

}
