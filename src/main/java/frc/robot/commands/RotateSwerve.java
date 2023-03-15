// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class RotateSwerve extends CommandBase {

  private SwerveSubsystem swerve;
  private double headingx;
  private double headingy;
  private double rotationSpeed;
  /**
   * bring the intake in or out
   * */
  public RotateSwerve(SwerveSubsystem swerve,double headingx,double headingy) {
    this.swerve = swerve;
    this.headingx = headingx;
    this.headingy = headingy;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // swerve.drive(new Translation2d(speed,0), 0, true,true);
  }

  @Override
  public void execute() {
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(0, 0,
      headingx,
      headingy,
      swerve.getHeading().getRadians());
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                             Constants.LOOP_TIME,
                                             Constants.CHASSIS_MASS, Constants.ROBOT_MASS, Constants.CHASSIS_CG,
                                             swerve.getSwerveDriveConfiguration());
    rotationSpeed = desiredSpeeds.omegaRadiansPerSecond;
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationSpeed < 0.5;
  }
}
