// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.MotorSpeedsSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeAndGrabberSpeedManager extends CommandBase {

    private MotorSpeedsSupplier intakeMotorSpeedsSupplier;
    private MotorSpeedsSupplier grabberMotorSpeedsSupplier;

    private IntakeSubsystem intake;
    private GrabberSubsystem grabber;

  /**
   * bring the intake in or out
   * */
  public IntakeAndGrabberSpeedManager(IntakeSubsystem intake, GrabberSubsystem grabber) {
    intakeMotorSpeedsSupplier = new MotorSpeedsSupplier();
    grabberMotorSpeedsSupplier = new MotorSpeedsSupplier();

    this.intake = intake;
    this.grabber = grabber;
    addRequirements(intake, grabber);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
