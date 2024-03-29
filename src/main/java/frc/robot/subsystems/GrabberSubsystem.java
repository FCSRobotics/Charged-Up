// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Grabber;

public class  GrabberSubsystem extends SubsystemBase
{

  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;
  private boolean in = true;
  private final DoubleSolenoid solenoid;

  public GrabberSubsystem(int rightMotorId, 
                int leftMotorId,
                int extendChannel, 
                int retractChannel
                ) {
    leftMotor = new CANSparkMax(leftMotorId,MotorType.kBrushless);
    leftMotor.setSmartCurrentLimit(10);
    rightMotor = new CANSparkMax(rightMotorId, MotorType.kBrushless);
    rightMotor.setSmartCurrentLimit(10);

    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, extendChannel, retractChannel);
  }

  // currently using percentage voltage might be better to change it to explicit speed
  public void setMotorsSpeeds(double speed, boolean in) {
    leftMotor.set(-speed);
    rightMotor.set(speed);
    this.in = in;
  }

  public void periodic() { 
    double current = leftMotor.getOutputCurrent();
    if (current >= Grabber.offCurrent && Grabber.disableMotors && in) {
      stopMotors();
    }
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void toggleMotorDirection() {
    setMotorsSpeeds( !in ? Grabber.motorSpeeds : -Grabber.motorSpeeds, !in); 
  }

  public void clamp() {
    solenoid.set(kReverse);
  }

  public void unclamp() {
    solenoid.set(kForward);
  }
}
