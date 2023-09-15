// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import swervelib.SwerveDrive;

/**
 * An example command that uses an example subsystem.
 */
public class MoveTime extends CommandBase {

  private SwerveSubsystem swerve;
  private long endTime;
  private final double vx;
  private final double vy;
  private final long milliseconds;

  /**
   * bring the intake in or out
   * */
  public MoveTime(SwerveSubsystem swerve,double vx, double vy, long milliseconds) {
    this.swerve = swerve;
    this.vx = vx;
    this.vy = vy;
    this.milliseconds = milliseconds;
    
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    endTime = System.currentTimeMillis() + milliseconds;
    swerve.drive(new Translation2d(vx,vy), 0, true,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() > endTime) {
      swerve.drive(new Translation2d(0,0),0,true,false);
      return true;
    }
    return false;
  }
}
