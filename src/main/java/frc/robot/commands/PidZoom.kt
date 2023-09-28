// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.swervedrive2.SwerveSubsystem
import frc.robot.utils.MovingAverage

/**
 * An example command that uses an example subsystem.
 */
class PidZoom(private val swerve: SwerveSubsystem, location: Double) : CommandBase() {
    private val pid: PIDController
    private val location: Double
    private val posMovingAverage: MovingAverage
    private val sensedMovingAverage: MovingAverage
    private var currentAverage = 0.0
    private var detect = 0.0
    private var inPosition = false
    private var table: NetworkTable? = null

    /**
     * Extend arm to given distance in meters
     *
     * @param armSubsystem      The arm subsystem
     * @param distanceMeters    Target distance in meters
     *
     * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
     */
    init {
        pid = PIDController(1.0, 0.0, 0.0)
        this.location = location
        posMovingAverage = MovingAverage(20)
        sensedMovingAverage = MovingAverage(20)
        addRequirements(swerve)
    }

    override fun initialize() {
        inPosition = false
        SmartDashboard.putString("moving forvards", "nope")
        swerve.drive(Translation2d(1.0, 0.0), 0.0, true, true)
        table = NetworkTableInstance.getDefault().getTable("limelight")
        val ta = table?.getEntry("ta")?.getDouble(0.0) ?: 0.0
        sensedMovingAverage.setAll(ta)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        table = NetworkTableInstance.getDefault().getTable("limelight")
        val ta = table?.getEntry("ta")?.getDouble(0.0) ?: 0.0
        val tv = table?.getEntry("tv")?.getDouble(0.0) ?: 0.0
        currentAverage = posMovingAverage.addValue(ta)
        detect = sensedMovingAverage.addValue(tv)
        SmartDashboard.putNumber("lime distance: ", Math.abs(currentAverage - location))

        //if (detect>0 & ta!=0){
        inPosition = if (ta > 0) {
            SmartDashboard.putString("moving forvards", "wheeeeee")
            swerve.drive(Translation2d(pid.calculate(ta, location), 0.0), 0.0, true, false)
            if (Math.abs(currentAverage - location) < 0.1) {
                true
            } else {
                false
            }
        } else {
            true
        }
        //}
        // else {
        //      inPosition = true;
        // }
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
