// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.Grabber
import frc.robot.subsystems.GrabberSubsystem
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * An example command that uses an example subsystem.
 */
class GrabberMotorsControl(private val grabberSubsystem: GrabberSubsystem, private val spinSpeed: DoubleSupplier, private val spinIn: BooleanSupplier) : CommandBase() {
    /**
     * Extend arm to given distance in meters
     *
     * @param grabber      The arm subsystem
     * @param distanceMeters    Target distance in meters
     *
     * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
     */
    init {
        addRequirements(grabberSubsystem)
    }

    override fun initialize() {
        SmartDashboard.putBoolean("grabber check", false)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if (spinSpeed.asDouble > 0.1) {
            grabberSubsystem.setMotorsSpeeds((if (spinIn.asBoolean) 1 else -1) * spinSpeed.asDouble * Grabber.maxSpeeds, spinIn.asBoolean)
        }
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
