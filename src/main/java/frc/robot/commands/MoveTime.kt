// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

/**
 * An example command that uses an example subsystem.
 */
class MoveTime(private val swerve: SwerveSubsystem, private val vx: Double, private val vy: Double, private val milliseconds: Long) : CommandBase() {
    private var endTime: Long = 0

    /**
     * bring the intake in or out
     */
    init {
        addRequirements(swerve)
    }

    override fun initialize() {
        endTime = System.currentTimeMillis() + milliseconds
        swerve.drive(Translation2d(vx, vy), 0.0, true, false)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        if (System.currentTimeMillis() > endTime) {
            swerve.drive(Translation2d(0.0, 0.0), 0.0, true, false)
            return true
        }
        return false
    }
}
