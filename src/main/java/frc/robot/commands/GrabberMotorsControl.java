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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.utils.ArmPosition;


/**
 * An example command that uses an example subsystem.
 */
public class GrabberMotorsControl extends CommandBase
{

  private final GrabberSubsystem grabberSubsystem;
  private DoubleSupplier spinSpeed;
  private BooleanSupplier spinIn;
  
  

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
  public GrabberMotorsControl(GrabberSubsystem grabber,DoubleSupplier spinSpeed, BooleanSupplier spinIn)
  {
    this.grabberSubsystem = grabber;
    this.spinSpeed = spinSpeed;
    this.spinIn = spinIn;
    
    addRequirements(grabber);
  }

  @Override
  public void initialize()
  { 
    SmartDashboard.putBoolean("grabber check", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(spinSpeed.getAsDouble() > 0.1){
      grabberSubsystem.setMotorsSpeeds((spinIn.getAsBoolean() ? 1 : -1) * spinSpeed.getAsDouble() * Grabber.maxSpeeds, spinIn.getAsBoolean() );
    }
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
