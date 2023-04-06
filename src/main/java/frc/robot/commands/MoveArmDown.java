// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.Position;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Grabber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.Positions;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import swervelib.SwerveDrive;

/**
 * An example command that uses an example subsystem.
 */
public class MoveArmDown extends CommandBase {

  private ArmSubsystem arm;
  private GrabberSubsystem grabber;
  private MotionLocation phase;
  private double[] positions;
  private double currentSum;
  private int currentLocationInList;

  public enum MotionLocation {
    MovingIn,
    GoingToLocation
  };

  /**
   * bring the intake in or out
   * */
  public MoveArmDown(ArmSubsystem arm,GrabberSubsystem grabber) {
    this.arm = arm;
    phase = MotionLocation.MovingIn;
    currentLocationInList = 0;
    currentSum = 0;
    this.grabber = grabber;
    positions = new double[Arm.rollingAverageLength];
    for (int i = 0; i < Arm.rollingAverageLength; i++) {
      positions[i] = 0;
    }
    
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setDesiredDistance(-1);
  }

  @Override
  public void execute() {
    DriverStation.reportWarning("arm extension auto: " + arm.getExtension(), false);
    DriverStation.reportWarning("arm rotation auto: " + arm.getRotation(), false);
    double newPosition = arm.getRotation();
    double oldPosition = positions[currentLocationInList];
    positions[currentLocationInList] = newPosition;
    currentSum -= oldPosition;
    currentSum += newPosition;
    currentLocationInList++;
    currentLocationInList = currentLocationInList % Arm.rollingAverageLength;
    double currentAverage = 360 - currentSum/Arm.rollingAverageLength;
    switch (phase) {
      case MovingIn:
        if (arm.getExtension() >= -0.2) {
          phase = MotionLocation.GoingToLocation;
          arm.setDesiredRotation(0);
          grabber.clamp();
        }
      break;
      case GoingToLocation:
      break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(360-arm.getRotation()) < 9) {
      return true;
    }
    return false;
  }
}
