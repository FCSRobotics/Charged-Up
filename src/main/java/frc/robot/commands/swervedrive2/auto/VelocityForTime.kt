// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swervedrive2.auto

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

/**
 * An example command that uses an example subsystem.
 */
class VelocityForTime(private val swerve: SwerveSubsystem, private val time: Double, private val velocity: Double) : CommandBase() {
    private val timer: Timer = Timer()

    /**
     * Extend arm to given distance in meters
     *
     * @param armSubsystem      The arm subsystem
     * @param distanceMeters    Target distance in meters
     *
     * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
     */
    init {
        addRequirements(swerve)
    }

    override fun initialize() {
        swerve.drive(Translation2d(0.5, 0.0), 0.0, true, true)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {}

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        swerve.drive(Translation2d(0.0, 0.0), 0.0, true, true)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return timer.hasElapsed(time)
    }
}
