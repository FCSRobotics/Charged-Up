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
import frc.robot.utils.MovingAverage;
import edu.wpi.first.networktables.*;

/**
 * An example command that uses an example subsystem.
 */
public class PidStrafe extends CommandBase
{

  private final SwerveSubsystem swerve;
  private PIDController pid;
  private double location;
  private MovingAverage posMovingAverage;
  private MovingAverage sensedMovingAverage;
  private double currentAverage;
  private double detect;
  private boolean inPosition;
  private NetworkTable table;
  
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
  public PidStrafe(SwerveSubsystem swerve, double location)
  {
    this.swerve = swerve;
    
    pid = new PIDController(0.1, 0, 0);
    this.location = location;
    this.posMovingAverage = new MovingAverage(20);
    this.sensedMovingAverage = new MovingAverage(20);
    
    addRequirements(swerve);
  }

  @Override
  public void initialize()
  { 
    SmartDashboard.putString("moving", "nope");
    inPosition = false;
    swerve.drive(new Translation2d(0,1), 0, true, true);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = table.getEntry("tx").getDouble(1);
    sensedMovingAverage.setAll(tx);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = table.getEntry("tx").getDouble(1);
    double tv = table.getEntry("tv").getDouble(1);
    SmartDashboard.putNumber("lime strafe: ", tx);
    currentAverage = posMovingAverage.addValue(Math.abs(tx-location));
    detect = sensedMovingAverage.addValue(tv);
    SmartDashboard.putNumber("detection: ", detect);
    
      SmartDashboard.putString("moving", "wheeeeee");
      //if (detect>0){
      swerve.drive(new Translation2d(0,-1*pid.calculate(location,tx)), 0, true, false);
      if (currentAverage<2) {
        inPosition = true;
      }
      else {
        inPosition = false;
      }
      
      // else {
      //   inPosition=true;
      // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    swerve.drive(new Translation2d(0,0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
  
    if (inPosition){
      return true;
    }
    return false;
  }

}
