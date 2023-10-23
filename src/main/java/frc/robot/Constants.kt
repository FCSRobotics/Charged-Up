// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import swervelib.parser.PIDFConfig

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {
    const val ROBOT_MASS = (148 - 20.3) * 0.453592 // 32lbs * kg per pound
    const val CHASSIS_MASS = ROBOT_MASS
    @JvmField
    val CHASSIS_CG = Translation3d(0.0, 0.0, Units.inchesToMeters(8.0))
    const val LOOP_TIME = 0.13 //s, 20ms + 110ms sprk max velocity lag

    object Auton {
        @JvmField
        val xAutoPID = PIDFConfig(0.7, 0.0, 0.0)
        @JvmField
        val yAutoPID = PIDFConfig(0.7, 0.0, 0.0)
        @JvmField
        val angleAutoPID = PIDFConfig(0.4, 0.0, 0.01)
        const val MAX_SPEED = 4.0
        const val MAX_ACCELERATION = 2.0
    }

    object Drivebase {
        // Hold time on motor brakes when disabled
        const val WHEEL_LOCK_TIME = 10.0 // seconds
    }

    object Intake {
        const val wheelTopSpeedCube = 0.1
        const val wheelBottomSpeedCube = 0.1
        const val wheelTopSpeedCone = 0.5
        const val wheelBottomSpeedCone = 0.5
        const val topMotor = 28
        const val bottomMotor = 27
        const val extendChannel = 12
        const val retractChannel = 11
        const val reverseSolenoid = false
        const val wheelBottomSpeedEject = 0.45
        const val wheelTopSpeedEject = 0.45
        const val maxTopSpeed = 1.0
        const val maxBottomSpeed = 1.0
        const val rotateMotor = -4 // TODO: Figure out actual number
    }

    object Grabber {
        const val rightMotorId = 24
        const val leftMotorId = 25
        const val extendChannel = 9
        const val retractChannel = 8
        const val offCurrent = -1.0
        const val disableMotors = false
        const val motorSpeeds = 0.5
        @JvmField
        var maxSpeeds = 1.0
    }

    object Arm {
        const val revToMetersConversionFactor = .01f // need to set
        const val extendSparkMaxId = 22
        const val revToAngleConversionFactor = 360f
        const val rotateSparkMaxId = 23
        const val iExtension = 0.00
        const val pExtension = 6.75
        const val dExtension = 0.0
        const val pRotating = 0.11
        const val iRotating = 0.0
        const val dRotating = 0.0
        const val extendCancoderid = -1
        const val rotateCancoderid = -1
        const val extendOffset = 0f
        const val rotateOffset = 0f
        const val maxSpeedExtend = 0.1
        const val maxSpeedRotate = 0.1
        const val maxAccelExtend = 0.1
        const val maxAccelRotate = 0.1
        const val rotateFollowSparkMaxId = 26
        const val rollingAverageLength = 20
        @JvmField
        val feedForwardMap = doubleArrayOf( // 0,
                // 0.005,
                // 0.005,
                // 0.006,
                // 0.006,
                // 0.006,
                0.0,
                0.140881,
                0.238044,
                0.388770,
                0.638903,
                0.681784) // public static final double[] feedForwardMap = {
        //   // 0,
        //   // 0.005,
        //   // 0.005,
        //   // 0.006,
        //   // 0.006,
        //   // 0.006,
        //   0,
        //   0,
        //   0,
        //   0,
        //   0,
        //   0,
        // };
    }

    object OperatorConstants {
        // Joystick Deadband
        const val LEFT_X_DEADBAND = 0.01
        const val LEFT_Y_DEADBAND = 0.01

        // public static final float MAX_INPUT_CHANGE = 10f;//5
        const val MAX_ANGLE_CHANGE = 30.0
        const val MAX_AMPLITUDE_CHANGE = 10.0
        const val MAX_INPUT_CHANGE = 10.0
    }

    object PIDBalance {
        // Joystick Deadband
        const val moveTime: Long = 750
        const val waitTime: Long = 500
    }
}
