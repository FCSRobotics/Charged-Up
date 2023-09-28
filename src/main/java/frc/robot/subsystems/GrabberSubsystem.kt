// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.Grabber

class GrabberSubsystem(rightMotorId: Int,
                       leftMotorId: Int,
                       extendChannel: Int,
                       retractChannel: Int
) : SubsystemBase() {
    // private final CANSparkMax leftMotor;
    // private final CANSparkMax rightMotor;
    private var `in` = true
    private val solenoid: DoubleSolenoid

    init {
        // leftMotor = new CANSparkMax(leftMotorId,MotorType.kBrushless);
        // leftMotor.setSmartCurrentLimit(10);
        // rightMotor = new CANSparkMax(rightMotorId, MotorType.kBrushless);
        // rightMotor.setSmartCurrentLimit(10);
        solenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, extendChannel, retractChannel)
    }

    // currently using percentage voltage might be better to change it to explicit speed
    fun setMotorsSpeeds(speed: Double, `in`: Boolean) {
        // leftMotor.set(-speed);
        // rightMotor.set(speed);
        this.`in` = `in`
    }

    override fun periodic() {
        // double current = leftMotor.getOutputCurrent();
        // if (current >= Grabber.offCurrent && Grabber.disableMotors && in) {
        // stopMotors();
        // }
    }

    fun stopMotors() {
        // leftMotor.stopMotor();
        // rightMotor.stopMotor();
    }

    fun toggleMotorDirection() {
        setMotorsSpeeds(if (!`in`) Grabber.motorSpeeds else -Grabber.motorSpeeds, !`in`)
    }

    fun clamp() {
        solenoid.set(DoubleSolenoid.Value.kReverse)
    }

    fun unclamp() {
        solenoid.set(DoubleSolenoid.Value.kForward)
    }

    fun zeroMotors() {
        setMotorsSpeeds(0.0, true)
    }
}
