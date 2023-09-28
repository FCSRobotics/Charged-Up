// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.GrabberSubsystem

/**
 * An example command that uses an example subsystem.
 */
class StopGrabberMotors(private val grabberSubsystem: GrabberSubsystem) : CommandBase() {
    /**
     * Extend arm to given distance in meters
     * might not matter if it is a cone or a cube but... it's there if it is needed
     */
    init {
        addRequirements(grabberSubsystem)
    }

    override fun initialize() {
        grabberSubsystem.stopMotors()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return true
    }
}
