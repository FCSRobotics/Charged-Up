// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class SetIntakePosition extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final boolean desiredPosition; 
  

  /**
   * bring the intake in or out
   * */
  public SetIntakePosition(IntakeSubsystem intake, boolean out) {
    desiredPosition = out;
    intakeSubsystem = intake;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    if (desiredPosition) {
      intakeSubsystem.extendOut();
    } else {
      intakeSubsystem.pullIn();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
