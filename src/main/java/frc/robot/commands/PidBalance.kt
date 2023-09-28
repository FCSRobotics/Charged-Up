// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.PIDBalance
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

/**
 * An example command that uses an example subsystem.
 */
class PidBalance(private val swerve: SwerveSubsystem, private val gyro: Pigeon2, private val intake: IntakeSubsystem, zero: Double) : CommandBase() {
    private val onStation = false
    private val pid: PIDController = PIDController(0.045, 0.0, 0.0)
    private var startTime: Long
    private val zero: Double

    /**
     * Extend arm to given distance in meters
     *
     * @param armSubsystem      The arm subsystem
     * @param distanceMeters    Target distance in meters
     *
     * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
     */
    init {
        startTime = System.currentTimeMillis()
        this.zero = zero
        addRequirements(swerve)
    }

    override fun initialize() {
        startTime = System.currentTimeMillis()
        swerve.drive(Translation2d(-0.5, 0.0), 0.0, true, true)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val tilt = gyro.pitch
        SmartDashboard.putNumber("gyro: ", tilt)
        // if (!onStation) {
        //   if (Math.abs(tilt + 90) > 2) {
        //     onStation = true;
        //     intake.extendOut();
        //   }
        // } else {
        if ((System.currentTimeMillis() - startTime) % (PIDBalance.moveTime + PIDBalance.waitTime) > PIDBalance.moveTime) {
            swerve.drive(Translation2d(0.0, 0.0), 0.0, true, true)
            swerve.brakeMotors()
            SmartDashboard.putString("moving", "no I am not moving")
        } else {
            SmartDashboard.putString("moving", "yes I am moving")
            swerve.drive(Translation2d(pid.calculate(tilt, zero), 0.0), 0.0, true, false)
        }
        // }
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
