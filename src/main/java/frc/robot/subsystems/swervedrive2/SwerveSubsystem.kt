// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.swervedrive2

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.SubsystemBase
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.math.SwerveKinematics2
import swervelib.parser.SwerveControllerConfiguration
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import java.io.File

class SwerveSubsystem : SubsystemBase {
    /**
     * Swerve drive object.
     */
    private val swerveDrive: SwerveDrive

    /**
     * Initialize [SwerveDrive] with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    constructor(directory: File?) {
        swerveDrive = try {
            SwerveParser(directory).createSwerveDrive()
        } catch (e: Exception) {
            throw RuntimeException(e)
        }
    }

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    constructor(driveCfg: SwerveDriveConfiguration?, controllerCfg: SwerveControllerConfiguration?) {
        swerveDrive = SwerveDrive(driveCfg, controllerCfg)
    }

    /**
     * The primary method for controlling the drivebase.  Takes a [Translation2d] and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   [Translation2d] that is the commanded linear velocity of the robot, in meters per
     * second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     * torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     * (field North) and positive y is torwards the left wall when looking through the driver station
     * glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     * relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     * @param isOpenLoop    Whether to use closed-loop velocity control.  Set to true to disable closed-loop.
     */
    fun drive(translation: Translation2d?, rotation: Double, fieldRelative: Boolean, isOpenLoop: Boolean) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop)
    }

    override fun periodic() {
        swerveDrive.updateOdometry()
    }

    override fun simulationPeriodic() {}
    fun brakeMotors() {
        setMotorBrake(true)
    }

    val kinematics: SwerveKinematics2
        /**
         * Get the swerve drive kinematics object.
         *
         * @return [SwerveKinematics2] of the swerve drive.
         */
        get() = swerveDrive.kinematics

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    fun resetOdometry(initialHolonomicPose: Pose2d?) {
        swerveDrive.resetOdometry(initialHolonomicPose)
    }

    val pose: Pose2d
        /**
         * Gets the current pose (position and rotation) of the robot, as reported by odometry.
         *
         * @return The robot's pose
         */
        get() = swerveDrive.pose

    /**
     * Set field-relative chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Field-relative.
     */
    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds?) {
        swerveDrive.setChassisSpeeds(chassisSpeeds)
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    fun setMotorBrake(brake: Boolean) {
        swerveDrive.setMotorBrake(brake)
    }

    val heading: Rotation2d
        /**
         * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
         *
         * @return The yaw angle
         */
        get() = swerveDrive.yaw

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput                     X joystick input for the robot to move in the X direction.
     * @param yInput                     Y joystick input for the robot to move in the Y direction.
     * @param headingX                   X joystick which controls the angle of the robot.
     * @param headingY                   Y joystick which controls the angle of the robot.
     * @param currentHeadingAngleRadians The current robot heading in radians.
     * @return [ChassisSpeeds] which can be sent to th Swerve Drive.
     */
    fun getTargetSpeeds(xInput: Double, yInput: Double, headingX: Double, headingY: Double,
                        currentHeadingAngleRadians: Double): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, currentHeadingAngleRadians)
    }

    val fieldVelocity: ChassisSpeeds
        /**
         * Gets the current field-relative velocity (x, y and omega) of the robot
         *
         * @return A ChassisSpeeds object of the current field-relative velocity
         */
        get() = swerveDrive.fieldVelocity
    val swerveController: SwerveController
        /**
         * Get the [SwerveController] in the swerve drive.
         *
         * @return [SwerveController] from the [SwerveDrive].
         */
        get() = swerveDrive.swerveController
    val swerveDriveConfiguration: SwerveDriveConfiguration
        /**
         * Get the [SwerveDriveConfiguration] object.
         *
         * @return The [SwerveDriveConfiguration] fpr the current drive.
         */
        get() = swerveDrive.swerveDriveConfiguration
}
