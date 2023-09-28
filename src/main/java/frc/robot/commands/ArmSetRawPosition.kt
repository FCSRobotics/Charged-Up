// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSubsystem
import frc.robot.utils.ArmPosition

/**
 * An example command that uses an example subsystem.
 */
class ArmSetRawPosition(private val armSubsystem: ArmSubsystem, distancMeters: Float, rotation: Float) : CommandBase() {
    private val distanceMeters: Double
    private val rotation: Double

    /**
     * Extend arm to given distance in meters
     *
     * @param armSubsystem      The arm subsystem
     * @param distanceMeters    Target distance in meters
     *
     * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
     */
    init {
        distanceMeters = distancMeters.toDouble()
        this.rotation = rotation.toDouble()
        addRequirements(armSubsystem)
    }

    override fun initialize() {
        val pos = ArmPosition(distanceMeters, rotation)
        armSubsystem.setRawPosition(pos)
    }

    // Called every time the scheduler runs while the command is scheduled.
    // @Override
    // public void execute()
    // {
    //   this.currentDistance = this.armSubsystem.extendEncoder.getPosition();
    //   this.armSubsystem.extendSparkMax.set(this.distanceMeters - this.currentDistance);
    // }
    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return true
    }
}
