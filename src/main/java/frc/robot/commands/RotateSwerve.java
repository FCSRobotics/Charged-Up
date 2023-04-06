// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class RotateSwerve extends CommandBase {

  private SwerveSubsystem swerve;
  private double theta;
  private double rotationSpeed;
  private Pigeon2 gyro;
  private PIDController pid;
  /**
   * bring the intake in or out
   * */
  public RotateSwerve(SwerveSubsystem swerve, double theta, Pigeon2 gyro) {
    this.swerve = swerve;
    this.theta= theta;
    pid = new PIDController(0.6, 0, 0);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // swerve.drive(new Translation2d(speed,0), 0, true,true);
  }

  @Override
  public void execute() {
    double currentRotation = gyro.getYaw();
    rotationSpeed = pid.calculate(currentRotation,theta);
    swerve.drive(new Translation2d(0,0), rotationSpeed, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
