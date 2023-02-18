// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class ArmExtend extends CommandBase
{

  private final ArmSubsystem armSubsystem;
  private double currentDistance;
  private double distanceMeters;
  private float acceptableDistanceMeters;
  private double approxExtensionTimeSeconds;
  private double increment;
  

  /**
   * Extend arm to given distance in meters
   *
   * @param armSubsystem      The arm subsystem
   * @param distanceMeters    Target distance in meters
   *                         
   * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
   *                          
   * 
   */
  public ArmExtend(ArmSubsystem armSubsystem, float distancMeters, float acceptableDistanceMeters)
  {
    this.armSubsystem = armSubsystem;
    this.currentDistance = 0.0;
    this.distanceMeters = distancMeters;
    this.acceptableDistanceMeters = acceptableDistanceMeters;
    

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize()
  {
    
    this.currentDistance = this.armSubsystem.extendEncoder.getPosition();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    this.currentDistance = this.armSubsystem.extendEncoder.getPosition();
    this.armSubsystem.extendSparkMax.set(this.distanceMeters - this.currentDistance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return Math.abs(this.distanceMeters - this.currentDistance) > 0.02;
  }


}
