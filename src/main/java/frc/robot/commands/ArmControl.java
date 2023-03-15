// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.ArmPosition;


/**
 * An example command that uses an example subsystem.
 */
public class ArmControl extends CommandBase
{

  private final ArmSubsystem armSubsystem;
  private DoubleSupplier rotation;
  private DoubleSupplier extensionSupplier;
  
  

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
  public ArmControl(ArmSubsystem armSubsystem,DoubleSupplier rotation, DoubleSupplier extensionSupplier)
  {
    this.armSubsystem = armSubsystem;
    this.extensionSupplier = extensionSupplier;
    this.rotation = rotation;
    
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize()
  { 
    SmartDashboard.putBoolean("check", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    armSubsystem.setRawPosition(new ArmPosition(extensionSupplier.getAsDouble(), rotation.getAsDouble()));
    SmartDashboard.putNumber("Angle ", rotation.getAsDouble());
    SmartDashboard.putBoolean("check", true);
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
    return false;
  }


}
