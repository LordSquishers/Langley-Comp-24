// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.MathMethods;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.CameraSubsystem;



public class PivotControllerCmd extends Command{

  private final PivotSubsystem pivotSubsystem;
  private final CameraSubsystem cameraSubsystem;
  private final Supplier<Double> pivotPositiveDirFunction;
  private final Supplier<Double> pivotNegativeDirFunction;

  public PivotControllerCmd(PivotSubsystem pivotSubsystem, Supplier<Double> pivotPositiveDirFunction, Supplier<Double> pivotNegativeDirFunction) {
    this.pivotSubsystem = pivotSubsystem;
    this.cameraSubsystem = null;
    this.pivotPositiveDirFunction = pivotPositiveDirFunction;
    this.pivotNegativeDirFunction = pivotNegativeDirFunction;

    addRequirements(pivotSubsystem);
  }

  // public PivotControllerCmd(PivotSubsystem pivotSubsystem, CameraSubsystem cameraSubsystem) {
  //   this.pivotSubsystem = pivotSubsystem;
  //   this.cameraSubsystem = cameraSubsystem;
  //   this.pivotPositiveDirFunction = () -> (MathMethods.speedMax(
  //     PivotConstants.kAprilCamSpeedFactor*Math.sin(Math.toRadians(cameraSubsystem.getPitch())),
  //     PivotConstants.kAprilCamMaxSpeedMetersPerSecond,
  //     PivotConstants.kAprilCamDeadbandDegrees,
  //     PivotConstants.kAprilCamMinSpeed, 
  //     () -> cameraSubsystem.getPitch()));

  //   this.pivotNegativeDirFunction = () -> (MathMethods.speedMax(
  //     PivotConstants.kAprilCamSpeedFactor*Math.sin(Math.toRadians(cameraSubsystem.getPitch())),
  //     PivotConstants.kAprilCamMaxSpeedMetersPerSecond,
  //     PivotConstants.kAprilCamDeadbandDegrees,
  //     PivotConstants.kAprilCamMinSpeed, 
  //     () -> cameraSubsystem.getPitch()));;

  //   addRequirements(pivotSubsystem, cameraSubsystem);
  // }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    
    //is the button pressed
    double positiveSpeed = pivotPositiveDirFunction.get();
    double negativeSpeed = pivotNegativeDirFunction.get();

    double velocity = positiveSpeed - negativeSpeed;

    pivotSubsystem.setPivotMotor(velocity);
  }


  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stopPivotMotor();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
