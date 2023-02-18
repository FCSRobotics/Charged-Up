// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveKinematics2;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class ArmSubsystem extends SubsystemBase
{

  public final CANSparkMax extendSparkMax;
  public final RelativeEncoder extendEncoder;

  public ArmSubsystem(int extendSparkMaxId, float revToMetersConversionFactor) {
    extendSparkMax = new CANSparkMax(extendSparkMaxId, MotorType.kBrushless);
    extendEncoder = extendSparkMax.getEncoder();
    extendEncoder.setPositionConversionFactor(revToMetersConversionFactor);
    extendEncoder.setVelocityConversionFactor(revToMetersConversionFactor);
  }
  // public void extendToDistanceMeters(float distance) {
      
  // }

}
