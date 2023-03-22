// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Positions;
import frc.robot.utils.MovingAverage;

/**
 * An example command that uses an example subsystem.
 */
public class MoveArmPosition extends CommandBase {

  private ArmSubsystem arm;
  private final Positions pos;
  private MotionLocation phase;
  private double currentAverage;

  private MovingAverage rotateMovingAverage;

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

    this.rotateMovingAverage = new MovingAverage(Arm.rollingAverageLength);
    phase = MotionLocation.MovingIn;
    
    
    
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setDesiredDistance(-1);
  }

  @Override
  public void execute() {
    DriverStation.reportWarning("arm extension auto: " + arm.getExtension(), false);
    DriverStation.reportWarning("arm rotation auto: " + arm.getRotation(),false);

    currentAverage = rotateMovingAverage.addValue(arm.getRotation());

    SmartDashboard.putNumber("current average of arm position", currentAverage);

    switch (phase) {
      case MovingIn:
        if (arm.getExtension() >= -2) {
          arm.setDesiredRotation(arm.getPostionAngle(pos));
          phase = MotionLocation.GoingToLocation;
        }
      break;
      case GoingToLocation:
        if (Math.abs(arm.getPostionAngle(pos) - currentAverage) < 4) {
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
