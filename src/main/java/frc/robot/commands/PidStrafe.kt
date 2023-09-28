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
class PidStrafe(private val swerve: SwerveSubsystem, location: Double) : CommandBase() {
    private val pid: PIDController = PIDController(0.1, 0.0, 0.0)
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
        this.location = location
        posMovingAverage = MovingAverage(20)
        sensedMovingAverage = MovingAverage(20)
        addRequirements(swerve)
    }

    override fun initialize() {
        SmartDashboard.putString("moving", "nope")
        inPosition = false
        swerve.drive(Translation2d(0.0, 1.0), 0.0, true, true)
        table = NetworkTableInstance.getDefault().getTable("limelight")
        val tx = table?.getEntry("tx")?.getDouble(1.0) ?: 1.0
        sensedMovingAverage.setAll(tx)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        table = NetworkTableInstance.getDefault().getTable("limelight")
        //val tx = table?.getEntry("tx")?.getDouble(1.0) ?: 1.0
        //val tv = table?.getEntry("tv")?.getDouble(1.0) ?: 1.0
        val botpose = table?.getEntry("botpose")?.getDoubleArray(DoubleArray(6)) ?:DoubleArray(6)
        //SmartDashboard.putNumber("lime strafe: ", tx)
        //currentAverage = posMovingAverage.addValue(Math.abs(tx - location))
        //detect = sensedMovingAverage.addValue(tv)
        //SmartDashboard.putNumber("detection: ", detect)
        //SmartDashboard.putString("moving", "wheeeeee")
        SmartDashboard.putNumber("xpos", botpose[0])
        SmartDashboard.putNumber("ypos", botpose[1])
        SmartDashboard.putNumber("rotation", botpose[6])
        //if (detect>0){
        //swerve.drive(Translation2d(0.0, -1 * pid.calculate(location, tx)), 0.0, true, false)
        //inPosition = if (currentAverage < 2) {
            //true
        //} else {
            //false
        //}

        // else {
        //   inPosition=true;
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
