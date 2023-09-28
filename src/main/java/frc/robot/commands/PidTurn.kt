// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.swervedrive2.SwerveSubsystem
import frc.robot.utils.MovingAverage

/**
 * An example command that uses an example subsystem.
 */
class PidTurn(private val swerve: SwerveSubsystem, private val gyro: Pigeon2, angle: Double) : CommandBase() {
    private val pid: PIDController
    private val angle: Double
    private val angleMovingAverage: MovingAverage
    private var currentAverage = 0.0
    private var inPosition = false
    private var `var` = 0.0

    /**
     * Extend arm to given distance in meters
     *
     * How close to the target position the final position of the arm must be in meters.
     *
     *
     */
    init {
        pid = PIDController(-0.15, -0.1, 0.0)
        this.angle = angle
        angleMovingAverage = MovingAverage(50)
        addRequirements(swerve)
    }

    override fun initialize() {
        inPosition = false
        angleMovingAverage.setAll(90.0) //this line seems problematic. Rotation is in radians per second and calling drive only temporarily ovverides standard input
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val currAngle = ((gyro.yaw + 180 - angle) % 360 + 360) % 360 - 180
        SmartDashboard.putNumber("gyro distance: ", currAngle)
        currentAverage = angleMovingAverage.addValue(Math.abs(currAngle))
        SmartDashboard.putString("spinning", "wheeeeee")
        `var` = pid.calculate(currAngle, -4.0)
        SmartDashboard.putNumber("pid spiiin", `var`)
        swerve.drive(Translation2d(0.0, 0.0), `var`, true, false) //this line seems problematic because of the second reason listed above
        inPosition = if (currentAverage < 1) {
            true
        } else {
            false
        }
        SmartDashboard.putBoolean("ready", inPosition)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        swerve.drive(Translation2d(0.0, 0.0), 0.0, true, true)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return if (inPosition) {
            true
        } else false
    }
}
