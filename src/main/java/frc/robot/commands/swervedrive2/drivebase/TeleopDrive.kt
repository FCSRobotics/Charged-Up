// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swervedrive2.drivebase

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.swervedrive2.SwerveSubsystem
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * An example command that uses an example subsystem.
 */
class TeleopDrive(private val swerve: SwerveSubsystem, private val vX: DoubleSupplier, private val vY: DoubleSupplier, private val omega: DoubleSupplier,
                  private val driveMode: BooleanSupplier, private val isOpenLoop: Boolean) : CommandBase() {
    private val controller: SwerveController

    /**
     * Creates a new ExampleCommand.
     *
     * @param swerve The subsystem used by this command.
     */
    init {
        controller = swerve.swerveController

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val xVelocity = Math.pow(vX.asDouble, 3.0) * controller.config.maxSpeed
        val yVelocity = Math.pow(vY.asDouble, 3.0) * controller.config.maxSpeed
        val angVelocity = Math.pow(omega.asDouble, 3.0) * controller.config.maxAngularVelocity
        SmartDashboard.putNumber("vX", xVelocity)
        SmartDashboard.putNumber("vY", yVelocity)
        SmartDashboard.putNumber("omega", angVelocity)
        swerve.drive(
                Translation2d(
                        xVelocity,
                        yVelocity),
                angVelocity,
                driveMode.asBoolean,
                isOpenLoop)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
