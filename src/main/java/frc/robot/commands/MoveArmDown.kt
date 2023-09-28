// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.Arm
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.GrabberSubsystem
import kotlin.math.abs

/**
 * An example command that uses an example subsystem.
 */
class MoveArmDown(private val arm: ArmSubsystem, private val grabber: GrabberSubsystem) : CommandBase() {
    private var phase: MotionLocation
    private val positions: DoubleArray
    private var currentSum = 0.0
    private var currentLocationInList = 0

    enum class MotionLocation {
        MovingIn,
        GoingToLocation
    }

    /**
     * bring the intake in or out
     */
    init {
        phase = MotionLocation.MovingIn
        positions = DoubleArray(Arm.rollingAverageLength)
        for (i in 0 until Arm.rollingAverageLength) {
            positions[i] = 0.0
        }
        addRequirements(arm)
    }

    override fun initialize() {
        arm.desiredDistance = -1.0
    }

    override fun execute() {
        DriverStation.reportWarning("arm extension auto: " + arm.extension, false)
        DriverStation.reportWarning("arm rotation auto: " + arm.rotation, false)
        val newPosition = arm.rotation
        val oldPosition = positions[currentLocationInList]
        positions[currentLocationInList] = newPosition
        currentSum -= oldPosition
        currentSum += newPosition
        currentLocationInList++
        currentLocationInList %= Arm.rollingAverageLength
        val currentAverage = 360 - currentSum / Arm.rollingAverageLength
        when (phase) {
            MotionLocation.MovingIn -> if (arm.extension >= -0.2) {
                phase = MotionLocation.GoingToLocation
                arm.desiredHeight = 0.0
                grabber.clamp()
            }

            MotionLocation.GoingToLocation -> {}
        }
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return abs(360 - arm.rotation) < 9
    }
}
