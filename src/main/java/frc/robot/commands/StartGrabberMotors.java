// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class StartGrabberMotors extends CommandBase
{
  private final GrabberSubsystem grabberSubsystem;
  private final double motorSpeed;
  private final boolean in;

  /**
   * Extend arm to given distance in meters
   * might not matter if it is a cone or a cube but... it's there if it is needed
   */
  public StartGrabberMotors(GrabberSubsystem g, double speed, boolean in)
  {
    this.in = in;
    grabberSubsystem = g;
    motorSpeed = speed;
    addRequirements(grabberSubsystem);


  }

  @Override
  public void initialize() {
    grabberSubsystem.setMotorsSpeeds(motorSpeed,in);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
