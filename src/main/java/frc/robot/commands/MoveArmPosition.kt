// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.Arm
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ArmSubsystem.Positions
import frc.robot.utils.MovingAverage
import kotlin.math.abs

/**
 * An example command that uses an example subsystem.
 */
class MoveArmPosition(private val arm: ArmSubsystem, private val pos: Positions) : CommandBase() {
    private var phase: MotionLocation
    private var currentAverage = 0.0
    private val rotateMovingAverage = MovingAverage(Arm.rollingAverageLength)

    enum class MotionLocation {
        MovingIn,
        GoingToLocation,
        Extending
    }

    /**
     * bring the intake in or out
     */
    init {
        phase = MotionLocation.MovingIn
        addRequirements(arm)
    }

    override fun initialize() {
        arm.desiredDistance = 0.0
    }

    override fun execute() {
        DriverStation.reportWarning("arm extension auto: " + arm.extension, false)
        DriverStation.reportWarning("arm rotation auto: " + arm.rotation, false)
        currentAverage = 360 - rotateMovingAverage.addValue(arm.rotation)
        SmartDashboard.putNumber("current average of arm position", currentAverage)
        when (phase) {
            MotionLocation.MovingIn -> if (arm.extension >= -0.2) {
                arm.desiredHeight = arm.getPostionAngle(pos)
                phase = MotionLocation.GoingToLocation
            }

            MotionLocation.GoingToLocation -> if (Math.abs(arm.getPostionAngle(pos) - currentAverage) < 4) {
                phase = MotionLocation.Extending
                arm.desiredDistance = arm.getPostionExtension(pos)
            }

            MotionLocation.Extending -> {}
        }
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return (phase == MotionLocation.Extending
                && abs(arm.extension + arm.getPostionExtension(pos)) < 0.04)
    }
}
