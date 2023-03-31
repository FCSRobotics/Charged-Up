// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive2.auto;

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
import frc.robot.commands.MoveArmDown;
import frc.robot.commands.MoveArmPosition;
import frc.robot.commands.MoveTime;
import frc.robot.commands.OpenGrabber;
import frc.robot.commands.PidBalance;
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

public final class Autos
{

  private Autos()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase exampleAuto(SwerveSubsystem swerve)
  {
    PathPlannerTrajectory example = PathPlanner.loadPath("balance",
                                                         new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION));
    return Commands.sequence(new FollowTrajectory(swerve, example, true));
  }

  public static CommandBase driveAndSpin(SwerveSubsystem swerve)
  {
    return Commands.sequence(
        new RepeatCommand(new InstantCommand(() -> swerve.drive(new Translation2d(1, 0), 5, true, true), swerve)));
  }

  public static CommandBase setActionsBalance(SwerveSubsystem swerve,IntakeSubsystem intake,ArmSubsystem arm, GrabberSubsystem grabber)
  {
    return Commands.sequence(dropOffCone(swerve, arm, grabber),
                             new MoveTime(swerve, -1, 0, 1000),
                             new SetIntakePosition(intake, true),
                             new WaitCommand(1),
                             new MoveTime(swerve, -1, 0, 2125),
                             new InstantCommand(swerve::brakeMotors, swerve));
  }

  public static CommandBase gyroBalance(SwerveSubsystem swerve,IntakeSubsystem intake,ArmSubsystem arm, GrabberSubsystem grabber, Pigeon2 pigeon) {
    return Commands.sequence(//dropOffCone(swerve, arm, grabber),
                             new WithGyroBalance(swerve, pigeon, intake));
  }

  public static CommandBase pidbalance(SwerveSubsystem swerve,IntakeSubsystem intake,ArmSubsystem arm, GrabberSubsystem grabber, Pigeon2 pigeon) {
    return Commands.sequence(//dropOffCone(swerve, arm, grabber),
                             new MoveTime(swerve, -1, 0, 1000),
                             new SetIntakePosition(intake, true),
                             new MoveTime(swerve, -1, 0, 3000),
                             new SetIntakePosition(intake, false),
                             new PidBalance(swerve, pigeon, intake));
  }

  public static CommandBase leaveandbalance(SwerveSubsystem swerve,IntakeSubsystem intake,GrabberSubsystem grabber, ArmSubsystem arm)
  {
    return Commands.sequence(dropOffCone(swerve, arm, grabber),
                             new MoveTime(swerve, -1, 0, 1000),
                             new SetIntakePosition(intake, true),
                             new WaitCommand(1),
                             new MoveTime(swerve, -1, 0,2250),
                             new SetIntakePosition(intake, false),
                             new MoveTime(swerve, -1, 0, 2150),
                             new RotateSwerve(swerve, 0, -1),
                             new SetIntakePosition(intake, true));
                             
  }

  public static CommandBase leaveTheStadium(SwerveSubsystem swerve,ArmSubsystem arm, GrabberSubsystem grabber)
  {
    return Commands.sequence(//dropOffCone(swerve, arm, grabber),
                             new MoveTime(swerve, -1, 0, 1000+2250+2000));
  }
  public static CommandBase dropOffCone(SwerveSubsystem swerve,ArmSubsystem arm,GrabberSubsystem grabber) {
    return Commands.sequence(new InstantCommand(arm::bringIn),
                             new WaitCommand(1),
                            //  new InstantCommand(arm::setZeroPosition),
                             new StartGrabberMotors(grabber,-0.1, true),
                             new MoveArmPosition(arm,Positions.MIDDLE),
                            //  new StartGrabberMotors(grabber, 0.1, false),
                             new OpenGrabber(grabber),
                             new MoveArmDown(arm,grabber));
  }

  public static CommandBase pickUpConeCube(SwerveSubsystem swerve, ArmSubsystem arm, GrabberSubsystem grabber, IntakeSubsystem intake, boolean turnLeft) {
    return Commands.sequence(dropOffCone(swerve, arm, grabber),
                             new MoveTime(swerve, 0, turnLeft ? -1 : 1, 500),
                             new SetIntakePosition(intake, true),
                             new StartIntake(intake, true, true),
                             new MoveTime(swerve, -1, 0, 5000),
                            //  new RotateSwerve(swerve, turnLeft ? -1 : 1 ,0),
                            //  new RotateTime(swerve,0.1,1000),
                             new StopIntake(intake));
  }

  public static CommandBase nullAuto() {
    return Commands.sequence();
  }
}


//Auto 1: score, drive out for like 5 seconds, turn 90 degrees right, put arm in intake position, drive forwards while running intake
//Auto 2: score, drive out for like 5 seconds, turn 90 degrees left, put arm in intake position, drive forwards while running intake