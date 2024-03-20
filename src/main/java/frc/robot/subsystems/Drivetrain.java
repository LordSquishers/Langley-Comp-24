package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.TalonSwerve;
import frc.robot.subsystems.swerve.TalonSwerveModule;

import static frc.robot.Constants.Swerve.*;

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
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
