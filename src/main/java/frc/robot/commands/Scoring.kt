// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.*
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

class Scoring private constructor() {
    init {
        throw UnsupportedOperationException("This is a utility class!")
    }

    companion object {
        fun thirdLevelCube(thethingthatdoestheeating: IntakeSubsystem?): CommandBase {
            return Commands.sequence(
                    SetIntakePosition(thethingthatdoestheeating!!, true),
                    WaitCommand(0.6),
                    EjectIntake(thethingthatdoestheeating, false, false, -0.6f),
                    WaitCommand(0.1),
                    SetIntakePosition(thethingthatdoestheeating, false),
                    StopIntake(thethingthatdoestheeating)
            )
        }

        fun secondLevelCube(thethingthatdoestheeating: IntakeSubsystem?): CommandBase {
            return Commands.sequence(
                    SetIntakePosition(thethingthatdoestheeating!!, true),
                    WaitCommand(0.6),
                    EjectIntake(thethingthatdoestheeating, false, false, -0.5f),
                    WaitCommand(0.15),
                    SetIntakePosition(thethingthatdoestheeating, false),
                    StopIntake(thethingthatdoestheeating)
            )
        }

        fun shootCube(thethingthatdoestheeating: IntakeSubsystem?): Command {
            return Commands.sequence(
                    SetIntakePosition(thethingthatdoestheeating!!, true),
                    WaitCommand(0.65),
                    EjectIntake(thethingthatdoestheeating, false, false, -1f),
                    WaitCommand(0.05),
                    SetIntakePosition(thethingthatdoestheeating, false),
                    WaitCommand(0.6 - 0.05),
                    StopIntake(thethingthatdoestheeating)
            )
        }

        fun playerStation(drivebase: SwerveSubsystem, intake: IntakeSubsystem?): Command {
            return Commands.sequence(
                    MoveTime(drivebase, 0.0, -0.6, 250),
                    SetIntakePosition(intake!!, true),
                    InstantCommand({ drivebase.brakeMotors() }),
                    EjectIntake(intake, false, true, 0.75f)
            )
        }
    }
}
