// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import javax.swing.text.Position;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ArmSubsystem.Positions;
import frc.robot.utils.ArmPosition;
import frc.robot.utils.MovingAverage;


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
  public int lastFrameAngle;
  public int lastInputedAngle;
  public MovingAverage rotationMovingAverage;
  private ArmFeedforward currentArmFeedforward;


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
    this.grabberSubsystem = grabberSubsystem;
    lastFrameAngle = -1;
    lastInputedAngle = -1;
    rotationMovingAverage = new MovingAverage(Arm.rollingAverageLength);

    currentArmFeedforward = Arm.feedForwardMap[calculateIndexFromAngle(positionSupplier.getAsInt())];

    // positions = new double[Arm.rollingAverageLength];
    // for (int i = 0; i < Arm.rollingAverageLength; i++) {
    //   positions[i] = 0;
    // }
    // currentSum = 0;

    lastTimePositionHeld = System.currentTimeMillis();
    
    addRequirements(armSubsystem);
  }

  private int calculateIndexFromAngle(int angle) {
    
    return (int)Math.round(Math.floor(angle / 60));

  }

  @Override
  public void initialize()
  { 
    SmartDashboard.putBoolean("check", false);
    grabberSubsystem.clamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // armSubsystem.setPercentageex(extensionSupplier.getAsDouble());
    currentArmFeedforward = Arm.feedForwardMap[calculateIndexFromAngle(positionSupplier.getAsInt())];

    int angle = positionSupplier.getAsInt();

    if (angle != lastFrameAngle && lastFrameAngle == -1) {
      if (angle == lastInputedAngle) {
        lastInputedAngle = -1;
      } else {
        lastInputedAngle = angle;
      }
    }
    lastFrameAngle = angle;

    
    // double newPosition = armSubsystem.getRotation();
    // double oldPosition = positions[currentLocationInList];
    // positions[currentLocationInList] = newPosition;
    // currentSum -= oldPosition;
    // currentSum += newPosition;
    // currentLocationInList++;
    // currentLocationInList = currentLocationInList % Arm.rollingAverageLength;
    // double currentAverage = currentSum/Arm.rollingAverageLength;
    double currentAverage = rotationMovingAverage.addValue(armSubsystem.getRotation());
    SmartDashboard.putNumber("current average of arm position", currentAverage);


    int index = lastInputedAngle / 90;
    double deltaAngle = voltage.getAsDouble() * 20;
    double deltaExtension = extensionSupplier.getAsDouble() * 20;
    if (lastInputedAngle == -1) {
      long currentTime = System.currentTimeMillis();
      if (!intakeUp.getAsBoolean() && !returnToZero.getAsBoolean()) {
        if (lastTimePositionHeld + 1000 <= currentTime) {
          if (Math.abs(currentAverage - armSubsystem.getPostionAngle(Positions.PICKUP) - deltaAngle)  < 3) {
            armSubsystem.setDesiredDistance(armSubsystem.getPostionExtension(Positions.PICKUP) - deltaExtension);
            
          } else {
            armSubsystem.setDesiredRotation(armSubsystem.getPostionAngle(Positions.PICKUP) + deltaAngle);
          }
        } else {
          if (armSubsystem.getExtension() - deltaExtension >= -2) {
            armSubsystem.setPercentage(0);
            if (Math.abs(armSubsystem.getRotation()) < 1 ) {
              armSubsystem.stopMotors();
            }
          } else {
            lastTimePositionHeld = currentTime;
            armSubsystem.setDesiredDistance(-1);
            grabberSubsystem.clamp();
          }
        }
      } else {
        if (armSubsystem.getExtension() - deltaExtension >= -2) {
          armSubsystem.setPercentage(0);
          if (Math.abs(armSubsystem.getRotation()) < 1 ) {
            armSubsystem.stopMotors();
          }
        }
        armSubsystem.setDesiredDistance(-1);
        
        grabberSubsystem.clamp();
        lastTimePositionHeld = currentTime;
      }
    } else {
      // armSubsystem.setDesiredDistance(0);
      lastTimePositionHeld = System.currentTimeMillis();
      
      if (armSubsystem.getExtension() - deltaExtension >= -2) {
        armSubsystem.setDesiredRotation(armSubsystem.getPostionAngle((Positions.values()[index])) + deltaAngle);
      } else if (armSubsystem.leftSide(Positions.values()[index]) != armSubsystem.leftSide(armSubsystem.getPostionAngle(Positions.values()[index])  + deltaAngle)) {
        armSubsystem.setDesiredDistance(-1);
      }
      if (Math.abs(currentAverage - armSubsystem.getPostionAngle(Positions.values()[index]) - deltaAngle) < 4) {
        armSubsystem.setDesiredDistance(armSubsystem.getPostionExtension(Positions.values()[index]) - deltaExtension);
      }
    }

    //armSubsystem.setRawPosition(new ArmPosition(0, voltage.getAsDouble()));
    SmartDashboard.putNumber("Angle ", deltaAngle);
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
