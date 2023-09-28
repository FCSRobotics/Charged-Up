// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSubsystem
import frc.robot.utils.ArmPosition
import java.util.function.DoubleSupplier

/**
 * An example command that uses an example subsystem.
 */
class ArmControl(private val armSubsystem: ArmSubsystem, private val rotation: DoubleSupplier, private val extensionSupplier: DoubleSupplier) : CommandBase() {
    /**
     * Extend arm to given distance in meters
     *
     * @param armSubsystem      The arm subsystem
     * @param distanceMeters    Target distance in meters
     *
     * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
     */
    init {
        addRequirements(armSubsystem)
    }

    override fun initialize() {
        SmartDashboard.putBoolean("check", false)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        armSubsystem.setRawPosition(ArmPosition(extensionSupplier.asDouble, rotation.asDouble))
        SmartDashboard.putNumber("Angle ", rotation.asDouble)
        SmartDashboard.putBoolean("check", true)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
