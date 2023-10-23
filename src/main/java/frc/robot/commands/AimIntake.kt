// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.IntakeSubsystem

/**
 * An example command that uses an example subsystem.
 */
class AimIntake(private val intakeSubsystem: IntakeSubsystem, private val limelight: NetworkTable) : CommandBase() {
    //TODO: Figure out the actual ids

    private var hasEjected = false
    private val gridIds: List<Int> = if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) listOf(1) else listOf()
    init {
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {

    }

    override fun execute() {
        //get whether the limelight has a valid target
        if(limelight.getEntry("tv").getInteger(0) == 1L && gridIds.contains(limelight.getEntry("tid").getInteger(0).toInt())) {
            // Limelight has a target and it is a scoring target
            val pose = limelight.getEntry("targetpose_robotspace").getDoubleArray(arrayOf(0.0, 0.0, 0.0, 0.0, 0.0))
            val distance = pose[0]
            val height = pose[2]
            intakeSubsystem.aim(distance, height)
            if(intakeSubsystem.isStationary) {
                EjectIntake(intakeSubsystem, cone = false, rotateIn = false, 1.0f).schedule()
                hasEjected = true
            }
        }

    }


    override fun end(interrupted: Boolean) {
        intakeSubsystem.pullIn()
    }
    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return limelight.getEntry("tv").getInteger(0) == 0L || hasEjected
    }
}
