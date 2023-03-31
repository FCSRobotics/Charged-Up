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
import frc.robot.Constants.Priority;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.MotorSpeedsSupplier;

import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class EjectIntake extends CommandBase
{

  private final IntakeSubsystem intakeSubsystem;
  boolean cone;
  boolean rotateIn;
  private float ejectSpeed;
  private MotorSpeedsSupplier intakeMotorSpeedsSupplier;
  

  /**
   * Extend arm to given distance in meters
   * might not matter if it is a cone or a cube but... it's there if it is needed
   */
  public EjectIntake(IntakeSubsystem i,MotorSpeedsSupplier intakeMotorSpeedsSupplier, boolean isCone, boolean rotateIn, float ejectSpeed)
  {
    intakeSubsystem = i;
    cone = isCone;
    this.rotateIn = rotateIn;
    addRequirements(intakeSubsystem);
    this.ejectSpeed = ejectSpeed;
    this.intakeMotorSpeedsSupplier = intakeMotorSpeedsSupplier;
  }

  @Override
  public void initialize() {
    double invert = rotateIn ? 1 : -1;
    intakeMotorSpeedsSupplier.setSpeedWithPriority(ejectSpeed, Priority.EjectPriority);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
