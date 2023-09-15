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
public class PidZoom extends CommandBase
{

  private final SwerveSubsystem swerve;
  private PIDController pid;
  private final double location;
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
  public PidZoom(SwerveSubsystem swerve, double location)
  {
    this.swerve = swerve;
    
    pid = new PIDController(1, 0, 0);
    this.location = location;
    this.posMovingAverage = new MovingAverage(20);
    this.sensedMovingAverage = new MovingAverage(20);
    
    addRequirements(swerve);
  }

  @Override
  public void initialize()
  { 
    inPosition = false;
    SmartDashboard.putString("moving forvards", "nope");
    swerve.drive(new Translation2d(1,0), 0, true, true);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    double ta = table.getEntry("ta").getDouble(0);
    sensedMovingAverage.setAll(ta);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    double ta = table.getEntry("ta").getDouble(0);
    double tv = table.getEntry("tv").getDouble(0);
    currentAverage = posMovingAverage.addValue(ta);
    detect = sensedMovingAverage.addValue(tv);
    SmartDashboard.putNumber("lime distance: ", Math.abs(currentAverage-location));
    
    //if (detect>0 & ta!=0){
    if (ta>0){
      SmartDashboard.putString("moving forvards", "wheeeeee");
      swerve.drive(new Translation2d(pid.calculate(ta,location),0), 0, true, false);
      if (Math.abs(currentAverage-location)<0.1) {
        inPosition = true;
      }
      else {
        inPosition = false;
      }
    }
    else {
        inPosition = true;
    }
    //}
    // else {
    //      inPosition = true;
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
