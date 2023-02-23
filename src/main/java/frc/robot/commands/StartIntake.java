// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class StartIntake extends CommandBase
{

  private final IntakeSubsystem intakeSubsystem;
  boolean cone;
  

  /**
   * Extend arm to given distance in meters
   * might not matter if it is a cone or a cube but... it's there if it is needed
   */
  public StartIntake(IntakeSubsystem i, boolean isCone)
  {
    intakeSubsystem = i;
    cone = isCone;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setBottomMotorSpeed(cone ? Intake.wheelBottomSpeedCone : Intake.wheelBottomSpeedCube);
    intakeSubsystem.setTopMotorSpeed(cone ? Intake.wheelTopSpeedCone : Intake.wheelTopSpeedCube);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
