// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Positions;

/**
 * An example command that uses an example subsystem.
 */
public class MoveArmPosition extends CommandBase {

  private ArmSubsystem arm;
  private final Positions pos;
  private MotionLocation phase;
  private double[] positions;
  private double currentSum;
  private int currentLocationInList;
  private double currentAverage;

  public enum MotionLocation {
    MovingIn,
    GoingToLocation,
    Extending
  };

  /**
   * bring the intake in or out
   * */
  public MoveArmPosition(ArmSubsystem arm, Positions pos) {
    this.arm = arm;
    this.pos = pos;
    phase = MotionLocation.MovingIn;
    currentLocationInList = 0;
    currentSum = 0;
    positions = new double[Arm.rollingAverageLenght];
    for (int i = 0; i < Arm.rollingAverageLenght; i++) {
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
    double newPosition = arm.getRotation();
    double oldPosition = positions[currentLocationInList];
    positions[currentLocationInList] = newPosition;
    currentSum -= oldPosition;
    currentSum += newPosition;
    currentLocationInList++;
    currentLocationInList = currentLocationInList % Arm.rollingAverageLenght;
    currentAverage = currentSum/Arm.rollingAverageLenght;
    switch (phase) {
      case MovingIn:
        if (arm.getExtension() >= -2) {
          arm.setDesiredRotation(arm.getPostionAngle(pos));
          phase = MotionLocation.GoingToLocation;
        }
      break;
      case GoingToLocation:
        if (Math.abs(arm.getPostionAngle(pos) - currentAverage) < 1) {
          phase = MotionLocation.Extending;
          arm.setDesiredDistance(arm.getPostionExtension(pos));
        }
      break;
      case Extending:
      break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (phase == MotionLocation.Extending
      && Math.abs(arm.getExtension() - arm.getPostionExtension(pos)) < 4) {
      return true;
    }
    return false;
  }
}
