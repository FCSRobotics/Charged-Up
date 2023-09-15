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


/**
 * An example command that uses an example subsystem.
 */
public class PidTurn extends CommandBase
{

  private final SwerveSubsystem swerve;
  private final Pigeon2 gyro;
  private PIDController pid;
  private final double angle;
  private MovingAverage angleMovingAverage;
  private double currentAverage;
  private boolean inPosition;
  private double var;
  
  /**
   * Extend arm to given distance in meters
   *
   How close to the target position the final position of the arm must be in meters.
   *                          
   * 
   */
  public PidTurn(SwerveSubsystem swerve, Pigeon2 gyro, double angle)
  {
    this.swerve = swerve;
    this.gyro = gyro;
    pid = new PIDController(-0.15, -0.1, 0);
    this.angle = angle;
    this.angleMovingAverage = new MovingAverage(50);
    
    addRequirements(swerve);
  }

  @Override
  public void initialize()
  { 
    inPosition = false;
    angleMovingAverage.setAll(90);//this line seems problematic. Rotation is in radians per second and calling drive only temporarily ovverides standard input
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currAngle = (((gyro.getYaw()+180-angle)%360+360)%360)-180;
    SmartDashboard.putNumber("gyro distance: ", currAngle);
    currentAverage = angleMovingAverage.addValue(Math.abs(currAngle));
    
      SmartDashboard.putString("spinning", "wheeeeee");
      var = pid.calculate(currAngle,-4);
      SmartDashboard.putNumber("pid spiiin", var);
      swerve.drive(new Translation2d(0,0), var, true, false); //this line seems problematic because of the second reason listed above
      if (currentAverage<1) {
        inPosition = true;
      }
      else {
        inPosition = false;
      }
      SmartDashboard.putBoolean("ready", inPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    swerve.drive(new Translation2d(0,0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    if (inPosition){
      return true;
    }
    return false;
  }

}
