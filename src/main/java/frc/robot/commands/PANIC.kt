// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

/**
 * An example command that uses an example subsystem.
 */
class PANIC(private val swerve: SwerveSubsystem, private val gyro: Pigeon2, private val zeroPitch: Double, private val zeroRoll: Double) : CommandBase() {
    private val pid: PIDController? = null
    private val startTime: Long = 0

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

    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val pitch = gyro.pitch
        val roll = gyro.roll
        swerve.drive(Translation2d((if (pitch - zeroPitch > 0) -1 else 1).toDouble(), (if (roll - zeroRoll > 0) -1 else 1).toDouble()), 0.0, false, false)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        swerve.drive(Translation2d(0.0, 0.0), 0.0, true, true)
        swerve.brakeMotors()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
