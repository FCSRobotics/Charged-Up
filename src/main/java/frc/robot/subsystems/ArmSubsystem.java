// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase
{

  public final CANSparkMax extendSparkMax;
  public final CANSparkMax rotateSparkMax;
  public final RelativeEncoder extendEncoder;
  public final RelativeEncoder rotatingEncoder;
  public double desiredDistance;
  public double currentDistance;
  public double desiredHeight;
  public double currentHeight;
  public final PIDController extensionPID;
  public final PIDController rotatingPID;

  public ArmSubsystem(int extendSparkMaxId, int rotateSparkMaxId, float revToMetersConversionFactor) {
    extendSparkMax = new CANSparkMax(extendSparkMaxId, MotorType.kBrushless);
    rotateSparkMax = new CANSparkMax(rotateSparkMaxId, MotorType.kBrushless);

    extendEncoder = extendSparkMax.getEncoder();
    extendEncoder.setPositionConversionFactor(revToMetersConversionFactor);
    extendEncoder.setVelocityConversionFactor(revToMetersConversionFactor);
    rotatingEncode = rotateSparkMax.getEncoder();
    rotatingEncoder.setPositionConversionFactor(revToMetersConversionFactor);

    desiredDistance = 0;
    currentDistance = extendEncoder.getPosition();
    extensionPID = new PIDController(Arm.pExtension, Arm.iExtension, Arm.dExtension);
    rotatingPID = new PIDController(Arm.pRotating, Arm.iRotating, Arm.dRotating);
  }

  @Override
  public void periodic() {
    if (Math.abs(desiredDistance - currentDistance) > 0.02) {
      currentDistance = extendEncoder.getPosition();
      extendSparkMax.set(extensionPID.calculate(currentDistance, desiredDistance)); // might want to change this to the built in one
    }
    if (Math.abs(desiredHeight - currentHeight) > 1) {
      currentHeight = rotatingEncoder.getPosition();
    }
  }

  public void setDesiredDistance(double distance) {
    desiredDistance = distance;
  } 

  public void setDesiredHeight(double height) {
    desiredHeight = height;
  }

  // public void extendToDistanceMeters(float distance) {
      
  // }
}
