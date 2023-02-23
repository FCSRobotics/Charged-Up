// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase
{

  public final CANSparkMax extendSparkMax;
  public final RelativeEncoder extendEncoder;
  public double desiredDistance;
  public double currentDistance;
  public final PIDController pid;

  public ArmSubsystem(int extendSparkMaxId, float revToMetersConversionFactor) {
    extendSparkMax = new CANSparkMax(extendSparkMaxId, MotorType.kBrushless);
    extendEncoder = extendSparkMax.getEncoder();
    extendEncoder.setPositionConversionFactor(revToMetersConversionFactor);
    extendEncoder.setVelocityConversionFactor(revToMetersConversionFactor);
    desiredDistance = 0;
    currentDistance = extendEncoder.getPosition();
    pid = new PIDController(Arm.p, Arm.i, Arm.d);
  }

  @Override
  public void periodic() {
    if (Math.abs(desiredDistance - currentDistance) > 0.02) {
      currentDistance = extendEncoder.getPosition();
      extendSparkMax.set(pid.calculate(currentDistance, desiredDistance));
    }
  }

  public void setDesiredDistance(double distance) {
    desiredDistance = distance;
  } 

  // public void extendToDistanceMeters(float distance) {
      
  // }
}
