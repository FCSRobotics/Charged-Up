// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Positions;
import frc.robot.utils.ArmPosition;


/**
 * An example command that uses an example subsystem.
 */
public class ArmControl2 extends CommandBase
{

  private final ArmSubsystem armSubsystem;
  private DoubleSupplier voltage;
  private DoubleSupplier extensionSupplier;
  private IntSupplier positionSupplier;

  /**
   * Extend arm to given distance in meters
   *
   * @param armSubsystem      The arm subsystem
   * @param distanceMeters    Target distance in meters
   *                         
   * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
   *                          
   * 
   */
  public ArmControl2(ArmSubsystem armSubsystem,DoubleSupplier voltage,DoubleSupplier extensionSupplier,IntSupplier povSupplier)
  {
    this.armSubsystem = armSubsystem;
    this.voltage = voltage;
    this.extensionSupplier = extensionSupplier;
    this.positionSupplier = povSupplier;

    
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize()
  { 
    SmartDashboard.putBoolean("check", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    armSubsystem.setPercentageex(extensionSupplier.getAsDouble());

    int angle = positionSupplier.getAsInt();

    //if (angle == -1) {
      armSubsystem.setPercentage(voltage.getAsDouble());
    //}
    // } else {
    //   int index = angle / 45;
    //   armSubsystem.setPosition(Positions.values()[index]);
    // }

    // armSubsystem.setRawPosition(new ArmPosition(0, voltage.getAsDouble()));
    // SmartDashboard.putNumber("Angle ", voltage.getAsDouble());
    // SmartDashboard.putBoolean("check", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}
