// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swervedrive2.auto

import com.ctre.phoenix.sensors.Pigeon2
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants.Auton
import frc.robot.commands.*
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ArmSubsystem.Positions
import frc.robot.subsystems.GrabberSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

class Autos private constructor() {
    init {
        throw UnsupportedOperationException("This is a utility class!")
    }

    companion object {
        /**
         * Example static factory for an autonomous command.
         */
        fun exampleAuto(swerve: SwerveSubsystem): CommandBase {
            val example = PathPlanner.loadPath("testpath", PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION))
            return Commands.sequence(FollowTrajectory(swerve, example, true))
        }

        fun driveAndSpin(swerve: SwerveSubsystem): CommandBase {
            return Commands.sequence(
                    RepeatCommand(InstantCommand({ swerve.drive(Translation2d(1.0, 0.0), 5.0, true, true) }, swerve)))
        }

        fun setActionsBalance(swerve: SwerveSubsystem, intake: IntakeSubsystem?, arm: ArmSubsystem?, grabber: GrabberSubsystem?): CommandBase {
            return Commands.sequence(dropOffCone(swerve, arm, grabber),
                    MoveTime(swerve, -1.0, 0.0, 1000),
                    SetIntakePosition(intake!!, true),
                    WaitCommand(1.0),
                    MoveTime(swerve, -1.0, 0.0, 2125),
                    InstantCommand({ swerve.brakeMotors() }, swerve))
        }

        fun gyroBalance(swerve: SwerveSubsystem, intake: IntakeSubsystem, arm: ArmSubsystem?, grabber: GrabberSubsystem?, pigeon: Pigeon2): CommandBase {
            return Commands.sequence(dropOffCone(swerve, arm, grabber),
                    WithGyroBalance(swerve, pigeon, intake))
        }

        fun pidbalance(swerve: SwerveSubsystem?, intake: IntakeSubsystem?, arm: ArmSubsystem?, grabber: GrabberSubsystem?, pigeon: Pigeon2): CommandBase {
            val zero = pigeon.pitch
            return Commands.sequence(dropOffCone(swerve, arm, grabber),
                    MoveTime(swerve!!, -2.0, 0.0, 550),
                    SetIntakePosition(intake!!, true),
                    MoveTime(swerve, -1.0, 0.0, 2500),
                    SetIntakePosition(intake, false),
                    PidBalance(swerve, pigeon, intake, zero))
        }

        fun leaveandbalance(swerve: SwerveSubsystem?, intake: IntakeSubsystem?, grabber: GrabberSubsystem?, arm: ArmSubsystem?, gyro: Pigeon2): CommandBase {
            val zero = gyro.pitch
            return Commands.sequence(dropOffCone(swerve, arm, grabber),
                    MoveTime(swerve!!, -1.0, 0.0, 1000),
                    SetIntakePosition(intake!!, true),
                    WaitCommand(1.0),
                    MoveTime(swerve, -1.0, 0.0, 2250),
                    SetIntakePosition(intake, false),
                    MoveTime(swerve, -1.0, 0.0, 1500),
                    RotateTime(swerve, Math.PI, 1150),
                    SetIntakePosition(intake, true),
                    MoveTime(swerve, 1.0, 0.0, 2000),
                    SetIntakePosition(intake, false),
                    PidBalance(swerve, gyro, intake, zero))
        }

        fun leaveTheStadium(swerve: SwerveSubsystem?, arm: ArmSubsystem?, grabber: GrabberSubsystem?, intake: IntakeSubsystem?): CommandBase {
            return Commands.sequence(dropOffCone(swerve, arm, grabber),
                    SetIntakePosition(intake!!, true),
                    StartIntake(intake, true, true),
                    MoveTime(swerve!!, -1.0, 0.0, (1000 + 2250 + 2000 - 250).toLong()))
        }

        fun dropOffCone(swerve: SwerveSubsystem?, arm: ArmSubsystem?, grabber: GrabberSubsystem?): CommandBase {
            return Commands.sequence( //  new InstantCommand(arm::setZeroPosition),
                    //  new StartGrabberMotors(grabber,-0.1, true),
                    MoveArmPosition(arm!!, Positions.UP),  //  new StartGrabberMotors(grabber, 0.1, false),
                    OpenGrabber(grabber!!),
                    WaitCommand(0.3),
                    MoveArmDown(arm, grabber))
        }

        fun anotherRandomThing(swerve: SwerveSubsystem?, arm: ArmSubsystem?, grabber: GrabberSubsystem?, intake: IntakeSubsystem?): CommandBase {
            return Commands.sequence(dropOffCone(swerve, arm, grabber),  // do not remove \/ else it will not work (this line satiates the ghost in the code)
                    MoveTime(swerve!!, 0.0, 0.0, (1000 + 2250 + 2000 - 1250).toLong()))
        }

        fun pickUpConeCube(swerve: SwerveSubsystem?, arm: ArmSubsystem?, grabber: GrabberSubsystem?, intake: IntakeSubsystem?, turnLeft: Boolean): CommandBase {
            return Commands.sequence(dropOffCone(swerve, arm, grabber),
                    MoveTime(swerve!!, 0.0, (if (turnLeft) -1 else 1).toDouble(), 500),
                    SetIntakePosition(intake!!, true),
                    StartIntake(intake, true, true),
                    MoveTime(swerve, -1.0, 0.0, 5000),  //  new RotateSwerve(swerve, turnLeft ? -1 : 1 ,0),
                    //  new RotateTime(swerve,0.1,1000),
                    StopIntake(intake))
        }

        fun nullAuto(): CommandBase {
            return Commands.sequence()
        }
    }
}
