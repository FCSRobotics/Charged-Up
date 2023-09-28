// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.swervedrive2.SwerveSubsystem

/**
 * An example command that uses an example subsystem.
 */
class RotateTime(private val swerve: SwerveSubsystem, private val rotationSpeed: Double, private val time: Long) : CommandBase() {
    private var startTime: Long = 0

    /**
     * bring the intake in or out
     */
    init {
        addRequirements(swerve)
    }

    override fun initialize() {
        // swerve.drive(new Translation2d(speed,0), 0, true,true);
        startTime = System.currentTimeMillis()
    }

    override fun execute() {
        // ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(0, 0,
        //   headingx,
        //   headingy,
        //   swerve.getHeading().getRadians());
        // Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        // translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        //                                          Constants.LOOP_TIME,
        //                                          Constants.CHASSIS_MASS, Constants.ROBOT_MASS, Constants.CHASSIS_CG,
        //                                          swerve.getSwerveDriveConfiguration());
        // rotationSpeed = desiredSpeeds.omegaRadiansPerSecond;
        swerve.drive(Translation2d(0.0, 0.0), rotationSpeed, true, false)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return System.currentTimeMillis() > startTime + time
    }
}
