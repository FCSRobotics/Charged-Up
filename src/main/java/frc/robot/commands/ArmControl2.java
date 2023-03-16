// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
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
  private double[] positions;
  private double currentSum;
  private int currentLocationInList;
  private BooleanSupplier intakeUp;
  private BooleanSupplier returnToZero;
  private long lastTimePositionHeld = 0;
  private GrabberSubsystem grabberSubsystem;


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
  public ArmControl2(ArmSubsystem armSubsystem,DoubleSupplier voltage,DoubleSupplier extensionSupplier,IntSupplier povSupplier,
                     BooleanSupplier intakeUp, BooleanSupplier returnToZero, GrabberSubsystem grabberSubsystem)
  {
    this.armSubsystem = armSubsystem;
    this.voltage = voltage;
    this.extensionSupplier = extensionSupplier;
    this.positionSupplier = povSupplier;
    this.intakeUp = intakeUp;
    this.returnToZero = returnToZero;

    positions = new double[Arm.rollingAverageLenght];
    for (int i = 0; i < Arm.rollingAverageLenght; i++) {
      positions[i] = 0;
    }
    currentSum = 0;

    lastTimePositionHeld = System.currentTimeMillis();
    
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
    // armSubsystem.setPercentageex(extensionSupplier.getAsDouble());

    int angle = positionSupplier.getAsInt();

    
    double newPosition = armSubsystem.getRotation();
    double oldPosition = positions[currentLocationInList];
    positions[currentLocationInList] = newPosition;
    currentSum -= oldPosition;
    currentSum += newPosition;
    currentLocationInList++;
    currentLocationInList = currentLocationInList % Arm.rollingAverageLenght;
    double currentAverage = currentSum/Arm.rollingAverageLenght;
    SmartDashboard.putNumber("current average of arm position", currentAverage);

    if (angle == -1) {
      long currentTime = System.currentTimeMillis();
      if (!intakeUp.getAsBoolean() && !returnToZero.getAsBoolean()) {
        if (lastTimePositionHeld + 1000 <= currentTime) {
          if (Math.abs(currentAverage - armSubsystem.getPostionAngle(Positions.PICKUP)) < 1) {
            armSubsystem.setDesiredDistance(armSubsystem.getPostionExtension(Positions.PICKUP));
            armSubsystem.setDesiredRotation(armSubsystem.getPostionAngle(Positions.PICKUP) + voltage.getAsDouble());
          } else {
            armSubsystem.setDesiredRotation(armSubsystem.getPostionAngle(Positions.PICKUP));
          }
        } else {
          if (armSubsystem.getExtension() >= -2) {
            armSubsystem.setPercentage(0);
          } else {
            lastTimePositionHeld = currentTime;
            armSubsystem.setDesiredDistance(-1);
            grabberSubsystem.clamp();
          }
        }
      } else {
        if (armSubsystem.getExtension() >= -2) {
          armSubsystem.setPercentage(0);
        }
        armSubsystem.setDesiredDistance(-1);
        grabberSubsystem.clamp();
        lastTimePositionHeld = currentTime;
      }
    } else {
      int index = angle / 90;
      // armSubsystem.setDesiredDistance(0);
      lastTimePositionHeld = System.currentTimeMillis();
      if (index == 3 && intakeUp.getAsBoolean()) {
        return;
      }
      if (armSubsystem.getExtension() >= -2) {
        armSubsystem.setDesiredRotation(armSubsystem.getPostionAngle((Positions.values()[index])));
      } else if (armSubsystem.leftSide(Positions.values()[index]) != armSubsystem.leftSide(armSubsystem.getPostionAngle(Positions.values()[index]))) {
        armSubsystem.setDesiredDistance(-1);
      }
      if (Math.abs(currentAverage - armSubsystem.getPostionAngle(Positions.values()[index])) < 1) {
        armSubsystem.setDesiredDistance(armSubsystem.getPostionExtension(Positions.values()[index]));
      }
    }

    //armSubsystem.setRawPosition(new ArmPosition(0, voltage.getAsDouble()));
    SmartDashboard.putNumber("Angle ", voltage.getAsDouble());
    SmartDashboard.putBoolean("check", true);
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
