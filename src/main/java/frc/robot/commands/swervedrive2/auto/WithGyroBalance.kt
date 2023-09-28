// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swervedrive2.auto

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

/**
 * An example command that uses an example subsystem.
 */
class WithGyroBalance(private val swerve: SwerveSubsystem, private val gyro: Pigeon2, private val intake: IntakeSubsystem) : CommandBase() {
    private var onStation = false

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
        swerve.drive(Translation2d(0.0, 0.5), 0.0, true, true)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val tilt = gyro.pitch
        if (!onStation) {
            if (Math.abs(tilt + 90) > 2) {
                onStation = true
                intake.extendOut()
            }
        } else {
            if (tilt < -91) {
                swerve.drive(Translation2d(0.2, 0.0), 0.0, true, false)
            } else if (tilt > -89) {
                swerve.drive(Translation2d(-0.2, 0.0), 0.0, true, false)
            } else {
                swerve.drive(Translation2d(0.0, 0.0), 0.0, true, false)
                swerve.brakeMotors()
            }
        }
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
