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

class IntakeSubsystem(topMotorId: Int,
                      bottomMotorId: Int,
                      extendChannel: Int,
                      retractChannel: Int,
                      reverseSolenoid: Boolean) : SubsystemBase() {
    private val bottomMotor: CANSparkMax
    private val topMotor: CANSparkMax
    private val solenoid: DoubleSolenoid
    private val reversed: Boolean
    var isIn = true
        private set
    private var intaking = true

    init {
        bottomMotor = CANSparkMax(bottomMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
        bottomMotor.restoreFactoryDefaults()
        bottomMotor.setSmartCurrentLimit(40)
        topMotor = CANSparkMax(topMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
        topMotor.restoreFactoryDefaults()
        topMotor.setSmartCurrentLimit(40)
        solenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, extendChannel, retractChannel)
        reversed = reverseSolenoid
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

    fun extendOut() {
        isIn = false
        solenoid.set(if (reversed) DoubleSolenoid.Value.kReverse else DoubleSolenoid.Value.kForward)
    }

    fun pullIn() {
        isIn = true
        solenoid.set(if (reversed) DoubleSolenoid.Value.kForward else DoubleSolenoid.Value.kReverse)
    }

    fun togglePosition() {
        if (isIn) {
            extendOut()
        } else {
            pullIn()
        }
    }

    fun zeroMotors() {
        setTopMotorSpeed(0.0)
        setBottomMotorSpeed(0.0)
    }
}
