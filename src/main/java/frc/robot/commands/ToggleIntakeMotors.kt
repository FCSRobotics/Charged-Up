// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.IntakeSubsystem

/**
 * An example command that uses an example subsystem.
 */
class ToggleIntakeMotors(private val intakeSubsystem: IntakeSubsystem) : CommandBase() {
    /**
     * bring the intake in or out
     */
    init {
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        intakeSubsystem.toggleIntakeMotors()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return true
    }
}
