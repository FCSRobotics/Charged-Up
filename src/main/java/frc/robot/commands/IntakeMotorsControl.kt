// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.Intake
import frc.robot.subsystems.IntakeSubsystem
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * An example command that uses an example subsystem.
 */
class IntakeMotorsControl(private val intake: IntakeSubsystem, private val spinSpeed: DoubleSupplier, private val spinIn: BooleanSupplier) : CommandBase() {
    /**
     * Extend arm to given distance in meters
     *
     * @param grabber      The arm subsystem
     * @param distanceMeters    Target distance in meters
     *
     * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
     */
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        SmartDashboard.putBoolean("check", false)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if (spinSpeed.asDouble > 0.1) {
            intake.setBottomMotorSpeed((if (spinIn.asBoolean) 1 else -1) * spinSpeed.asDouble * Intake.maxBottomSpeed)
            intake.setTopMotorSpeed((if (spinIn.asBoolean) 1 else -1) * spinSpeed.asDouble * Intake.maxTopSpeed)
        }
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
