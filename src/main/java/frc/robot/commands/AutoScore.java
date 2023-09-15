// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.Position;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Intake;
import frc.robot.commands.swervedrive2.auto.WithGyroBalance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.Positions;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;

public final class AutoScore
{

  private AutoScore()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */


   public static CommandBase ScoreHighRightCone(SwerveSubsystem swerve,Pigeon2 pigeon,ArmSubsystem arm, GrabberSubsystem grabber)
   {
     return Commands.sequence(new PidTurn(swerve, pigeon, 0),
                             new PidZoom(swerve,2.5),
                             new PidStrafe(swerve,17.1),
                             new PidTurn(swerve, pigeon, 0),
                             new MoveTime(swerve, 0, -1, 750),
                            new MoveArmPosition(arm,Positions.UP));
 
   }
  public static CommandBase ScoreHighLeftCone(SwerveSubsystem swerve,Pigeon2 pigeon,ArmSubsystem arm, GrabberSubsystem grabber)
  {
    return Commands.sequence(new PidTurn(swerve, pigeon, 0),
                            new PidZoom(swerve,2.5),
                            new PidStrafe(swerve,17.1),
                            new PidTurn(swerve, pigeon, 0),
                            new MoveTime(swerve, 0, 1, 750),
                             new MoveArmPosition(arm,Positions.UP));


  }

  public static CommandBase AllignHigh(SwerveSubsystem swerve,Pigeon2 pigeon,ArmSubsystem arm, GrabberSubsystem grabber)
  {
    return Commands.sequence(new PidTurn(swerve, pigeon, 0),
                            new PidStrafe(swerve,9));
                             

  }

  public static CommandBase AllignMid(SwerveSubsystem swerve,Pigeon2 pigeon,ArmSubsystem arm, GrabberSubsystem grabber)
  {
    return Commands.sequence(//new PidTurn(swerve, pigeon, 0),
                            new PidStrafe(swerve,11.6));
                             

  }

  
}


//Auto 1: score, drive out for like 5 seconds, turn 90 degrees right, put arm in intake position, drive forwards while running intake
//Auto 2: score, drive out for like 5 seconds, turn 90 degrees left, put arm in intake position, drive forwards while running intake
