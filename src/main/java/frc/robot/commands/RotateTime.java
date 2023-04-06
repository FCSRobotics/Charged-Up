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
public class RotateTime extends CommandBase {

  private SwerveSubsystem swerve;
  private long time;
  private double rotationSpeed;
  private long startTime;
  /**
   * bring the intake in or out
   * */
  public RotateTime(SwerveSubsystem swerve,double rotationSpeed,long time) {
    this.swerve = swerve;
    this.rotationSpeed = rotationSpeed;
    this.time = time;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // swerve.drive(new Translation2d(speed,0), 0, true,true);
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    // ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(0, 0,
    //   headingx,
    //   headingy,
    //   swerve.getHeading().getRadians());
    // Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    // translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
    //                                          Constants.LOOP_TIME,
    //                                          Constants.CHASSIS_MASS, Constants.ROBOT_MASS, Constants.CHASSIS_CG,
    //                                          swerve.getSwerveDriveConfiguration());
    // rotationSpeed = desiredSpeeds.omegaRadiansPerSecond;
    swerve.drive(new Translation2d(0,0), rotationSpeed, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() > (startTime + time);
  }
}
