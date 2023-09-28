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
class RotateSwerve(private val swerve: SwerveSubsystem, private val theta: Double, gyro: Pigeon2?) : CommandBase() {
    private var rotationSpeed = 0.0
    private val gyro: Pigeon2? = null
    private val pid: PIDController

    /**
     * bring the intake in or out
     */
    init {
        pid = PIDController(0.6, 0.0, 0.0)
        addRequirements(swerve)
    }

    override fun initialize() {
        // swerve.drive(new Translation2d(speed,0), 0, true,true);
    }

    override fun execute() {
        val currentRotation = gyro!!.yaw
        rotationSpeed = pid.calculate(currentRotation, theta)
        swerve.drive(Translation2d(0.0, 0.0), rotationSpeed, true, false)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return true
    }
}
