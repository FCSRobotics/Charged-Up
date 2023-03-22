// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive2.auto;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import frc.robot.utils.ArmPosition;
import swervelib.SwerveDrive;


/**
 * An example command that uses an example subsystem.
 */
public class WithGyroBalance extends CommandBase
{

  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;
  private final Pigeon2 gyro;
  private boolean onStation;
  
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
  public WithGyroBalance(SwerveSubsystem swerve, Pigeon2 gyro,IntakeSubsystem intake)
  {
    this.swerve = swerve;
    this.gyro = gyro;
    onStation = false;
    this.intake = intake;

    
    addRequirements(swerve);
  }

  @Override
  public void initialize()
  { 
    swerve.drive(new Translation2d(0,0.5), 0, true, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tilt = gyro.getPitch();
    if (!onStation) {
      if (Math.abs(tilt + 90) > 2) {
        onStation = true;
        intake.extendOut();
      }
    } else {
      if (tilt < -91) {
        swerve.drive(new Translation2d(0.2,0), 0, true, false);
      } else if (tilt > -89) {
        swerve.drive(new Translation2d(-0.2,0), 0, true, false);
      } else {
        swerve.drive(new Translation2d(0,0), 0, true, false);
        swerve.brakeMotors();
      }
    }
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
