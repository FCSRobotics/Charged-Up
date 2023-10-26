// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem
import jdk.nashorn.internal.objects.NativeMath.sqrt
import kotlin.math.pow

/**
 * An example command that uses an example subsystem.
 */
class AimIntake(private val intakeSubsystem: IntakeSubsystem, private val limelight: NetworkTable, private val scoringLevel: Constants.Intake.ScoringLevel) : CommandBase() {
    private var hasEjected = false





    init {
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {

    }

    override fun execute() {
        //get whether the limelight has a valid target
        if(limelight.getEntry("tv").getInteger(0) == 1L && Constants.Intake.gridIds.contains(limelight.getEntry("tid").getInteger(-1).toInt())) {
            // Limelight has a target and it is a scoring target
            val pose = limelight.getEntry("targetpose_robotspace").getDoubleArray(arrayOf(0.0, 0.0, 0.0, 0.0, 0.0))
            // X is robot forwards
            val distance = sqrt(pose[0].pow(2), pose[1].pow(2))
            // Z is vertical offset from robot position
            val height = pose[2] + (Constants.Intake.verticalOffsets[scoringLevel] ?: Constants.Intake.verticalOffsets[Constants.Intake.ScoringLevel.HIGH]!!) //This won't happen but if for some reason there is a missing scoring level it defaults to the high level
            intakeSubsystem.aim(distance, height)
            if(intakeSubsystem.isStationary && !hasEjected && intakeSubsystem.isCorrect()) {
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
