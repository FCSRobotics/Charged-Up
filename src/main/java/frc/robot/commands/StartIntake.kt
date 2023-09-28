// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.Intake
import frc.robot.subsystems.IntakeSubsystem

/**
 * An example command that uses an example subsystem.
 */
class StartIntake(private val intakeSubsystem: IntakeSubsystem, var cone: Boolean, var rotateIn: Boolean) : CommandBase() {
    /**
     * Extend arm to given distance in meters
     * might not matter if it is a cone or a cube but... it's there if it is needed
     */
    init {
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        val invert = (if (rotateIn) 1 else -1).toDouble()
        intakeSubsystem.setBottomMotorSpeed(invert * if (cone) Intake.wheelBottomSpeedCone else Intake.wheelBottomSpeedCube)
        intakeSubsystem.setTopMotorSpeed(invert * if (cone) Intake.wheelTopSpeedCone else Intake.wheelTopSpeedCube)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return true
    }
}
