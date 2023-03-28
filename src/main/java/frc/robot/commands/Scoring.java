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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Intake;
import frc.robot.commands.MoveArmDown;
import frc.robot.commands.MoveArmPosition;
import frc.robot.commands.MoveTime;
import frc.robot.commands.OpenGrabber;
import frc.robot.commands.RotateSwerve;
import frc.robot.commands.RotateTime;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.StartGrabberMotors;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopIntake;
import frc.robot.commands.ToggleGrabberMotors;
import frc.robot.commands.ToggleIntakeMotors;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.Positions;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;

public final class Scoring
{

  private Scoring()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static CommandBase thirdLevelCube(IntakeSubsystem thethingthatdoestheeating) {
    return Commands.sequence(
      new SetIntakePosition(thethingthatdoestheeating, true),
      new WaitCommand(0.55), 
      new EjectIntake(thethingthatdoestheeating, false, false, -0.5f),
      new WaitCommand(0.1),
      new SetIntakePosition(thethingthatdoestheeating, false),
      new StopIntake(thethingthatdoestheeating)
    );
  } 

  public static CommandBase secondLevelCube(IntakeSubsystem thethingthatdoestheeating) {
    return Commands.sequence(
      new SetIntakePosition(thethingthatdoestheeating, true),
      new WaitCommand(0.68), 
      new EjectIntake(thethingthatdoestheeating, false, false, -0.55f),
      new WaitCommand(0.05),
      new SetIntakePosition(thethingthatdoestheeating, false),
      new StopIntake(thethingthatdoestheeating)
    );
  }

public static Command shootCube(IntakeSubsystem thethingthatdoestheeating) {
  return Commands.sequence(
    new SetIntakePosition(thethingthatdoestheeating, true),
    new WaitCommand(0.65), 
    new EjectIntake(thethingthatdoestheeating, false, false, -1f),
    new WaitCommand(0.05),
    new SetIntakePosition(thethingthatdoestheeating, false),
    new WaitCommand(0.6 - 0.05),
    new StopIntake(thethingthatdoestheeating)
  );
}

  public static Command playerStation(SwerveSubsystem drivebase, IntakeSubsystem intake) {
    return Commands.sequence(
      new MoveTime(drivebase, 0, -0.6, 250),
      new SetIntakePosition(intake, true),
      new InstantCommand(drivebase::brakeMotors),
      new EjectIntake(intake, false, true, 0.75f)
    );
  }

  

}


//Auto 1: score, drive out for like 5 seconds, turn 90 degrees right, put arm in intake position, drive forwards while running intake
//Auto 2: score, drive out for like 5 seconds, turn 90 degrees left, put arm in intake position, drive forwards while running intake