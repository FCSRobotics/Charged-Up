// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Grabber;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Priority;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.ArmPosition;
import frc.robot.utils.MotorSpeedsSupplier;


/**
 * An example command that uses an example subsystem.
 */
public class IntakeMotorsControl extends CommandBase
{

  private final IntakeSubsystem intake;
  private DoubleSupplier spinSpeed;
  private BooleanSupplier spinIn;
private MotorSpeedsSupplier intakeMotorSpeedsSupplier;
  
  

  /**
   * Extend arm to given distance in meters
   *
   * @param grabber      The arm subsystem
   * @param distanceMeters    Target distance in meters
   *                         
   * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
   *                          
   * 
   */
  public IntakeMotorsControl(IntakeSubsystem intake,DoubleSupplier spinSpeed, BooleanSupplier spinIn, MotorSpeedsSupplier intakeMotorSpeedSupplier)
  {
    this.intake = intake;
    this.spinSpeed = spinSpeed;
    this.spinIn = spinIn;
    this.intakeMotorSpeedsSupplier = intakeMotorSpeedsSupplier;
    
    addRequirements(intake);
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
    
    intakeMotorSpeedsSupplier.setSpeedWithPriority((spinIn.getAsBoolean() ? 1 : -1) * spinSpeed.getAsDouble() * Intake.maxBottomSpeed, Priority.OperatorIntakeControl); 
    
    
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
