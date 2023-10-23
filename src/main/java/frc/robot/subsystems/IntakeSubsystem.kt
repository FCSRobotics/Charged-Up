// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.Intake
import frc.robot.utils.MovingAverage

class IntakeSubsystem(topMotorId: Int,
                      bottomMotorId: Int,
                      rotateMotorId: Int) : SubsystemBase() {
    private var stationaryUpdates = 0;
    var isStationary = false
    private var lastAngle = 0.0
    private val bottomMotor: CANSparkMax
    private val topMotor: CANSparkMax
    private val rotateMotor: CANSparkMax
    var isIn = true
        private set
    private var intaking = true

    init {
        bottomMotor = CANSparkMax(bottomMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
        topMotor = CANSparkMax(topMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
        rotateMotor = CANSparkMax(rotateMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
        lastAngle = rotateMotor.encoder.position / rotateMotor.encoder.countsPerRevolution

        //configure all of these because they have the same configurations.
        listOf(bottomMotor, topMotor, rotateMotor).forEach { configureMotor(it) }
    }

    private fun configureMotor(motor: CANSparkMax, smartCurrentLimit: Int = 40) {
        motor.restoreFactoryDefaults()
        motor.setSmartCurrentLimit(smartCurrentLimit)
    }

    // currently using percentage voltage might be better to change it to explicit speed
    fun setTopMotorSpeed(speed: Double) {
        topMotor.set(speed)
    }

    fun setBottomMotorSpeed(speed: Double) {
        bottomMotor.set(speed)
    }

    fun toggleIntakeMotors() {
        setTopMotorSpeed(if (intaking) Intake.wheelTopSpeedCone else -Intake.wheelTopSpeedCone)
        setBottomMotorSpeed(if (intaking) Intake.wheelBottomSpeedCone else -Intake.wheelBottomSpeedCone)
        intaking = !intaking
    }

    fun aim(distance: Double, height: Double) {
       // TODO: calculate the required angle
        val requiredAngle = distance
        setRotation(requiredAngle)

        val angle = rotateMotor.encoder.position / rotateMotor.encoder.countsPerRevolution

        if(lastAngle == angle && rotateMotor.encoder.velocity < 0.1) {
            stationaryUpdates++
        } else {
            stationaryUpdates = 0
        }

        isStationary = stationaryUpdates > 100


        lastAngle = angle


    }

    private fun setRotation(angle: Double) {
        rotateMotor.pidController.setReference(angle, CANSparkMax.ControlType.kPosition)
    }


    fun zeroMotors() {
        setTopMotorSpeed(0.0)
        setBottomMotorSpeed(0.0)
    }

    fun resetPosition() {
        setRotation(0.0)
    }


}
