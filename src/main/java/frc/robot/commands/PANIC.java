// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import javax.swing.plaf.basic.BasicTreeUI.TreeCancelEditingAction;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDBalance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import frc.robot.utils.ArmPosition;
import swervelib.SwerveDrive;


/**
 * An example command that uses an example subsystem.
 */
public class PANIC extends CommandBase
{

  private final SwerveSubsystem swerve;
  private final Pigeon2 gyro;
  private PIDController pid;
  private long startTime;  
  private double zeroPitch;
  private double zeroRoll;

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
  public PANIC(SwerveSubsystem swerve, Pigeon2 gyro, double zeroPitch,double zeroRoll)
  {
    this.swerve = swerve;
    this.gyro = gyro;
    this.zeroPitch = zeroPitch;
    this.zeroRoll = zeroRoll;
    
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = gyro.getPitch();
    double roll = gyro.getRoll();
    swerve.drive(new Translation2d((pitch-zeroPitch) > 0 ? -1 : 1, (roll-zeroRoll) > 0 ? -1 : 1), 0, false,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    swerve.drive(new Translation2d(0,0), 0, true, true);
    swerve.brakeMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}
