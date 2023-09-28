// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ArmSubsystem.Positions
import frc.robot.subsystems.GrabberSubsystem
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

class AutoScore private constructor() {
    init {
        throw UnsupportedOperationException("This is a utility class!")
    }

    companion object {
        /**
         * Example static factory for an autonomous command.
         */
        fun ScoreHighRightCone(swerve: SwerveSubsystem, pigeon: Pigeon2?, arm: ArmSubsystem, grabber: GrabberSubsystem?): CommandBase {
            return Commands.sequence(PidTurn(swerve, pigeon!!, 0.0),
                    PidZoom(swerve, 2.5),
                    PidStrafe(swerve, 17.1),
                    PidTurn(swerve, pigeon, 0.0),
                    MoveTime(swerve, 0.0, -1.0, 750),
                    MoveArmPosition(arm, Positions.UP))
        }

        fun ScoreHighLeftCone(swerve: SwerveSubsystem, pigeon: Pigeon2?, arm: ArmSubsystem, grabber: GrabberSubsystem?): CommandBase {
            return Commands.sequence(PidTurn(swerve, pigeon!!, 0.0),
                    PidZoom(swerve, 2.5),
                    PidStrafe(swerve, 17.1),
                    PidTurn(swerve, pigeon, 0.0),
                    MoveTime(swerve, 0.0, 1.0, 750),
                    MoveArmPosition(arm, Positions.UP))
        }

        fun AllignHigh(swerve: SwerveSubsystem?, pigeon: Pigeon2?, arm: ArmSubsystem?, grabber: GrabberSubsystem?): CommandBase {
            return Commands.sequence(PidTurn(swerve!!, pigeon!!, 0.0),
                    PidStrafe(swerve, 9.0))
        }

        fun AllignMid(swerve: SwerveSubsystem?, pigeon: Pigeon2?, arm: ArmSubsystem?, grabber: GrabberSubsystem?): CommandBase {
            return Commands.sequence( //new PidTurn(swerve, pigeon, 0),
                    PidStrafe(swerve!!, 11.6))
        }
    }
}
