// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.utils.ArmPosition;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class ArmSubsystem extends SubsystemBase
{

  public final CANSparkMax extendSparkMax;
  public final CANSparkMax rotateSparkMax;
  public final CANSparkMax rotateFollowSparkMax;
  public final RelativeEncoder extendEncoder;
  public final RelativeEncoder rotatingEncoder;
//   public final CANCoder extendCANCoder;
//   public final CANCoder rotateCANCoder;
  public double desiredDistance;
  public double currentDistance;
  public double desiredHeight;
  public double currentHeight;
  public final float extendOffset;
  public final float rotateOffset;



  public enum Positions {
    UP,
    MIDDLE,
    LOW,
    PICKUPSTATION,
    PICKUP,
    MIDDLE_CUBE,
    HIGH_CUBE
  };

  

  public final ArmPosition[] armPositions = {
    new ArmPosition(-85, 93.142197), //high back
    new ArmPosition(-56.35, 80.01), //mid back
    new ArmPosition(-51, 34), //low back
    new ArmPosition(0, 74.142250), // change this to back
    new ArmPosition(0,0), // low cube
  };

  public ArmSubsystem(int extendSparkMaxId, 
                      int rotateSparkMaxId,
                      int rotateSparkMaxFollowId, 
                      float extendOffset,
                      float rotateOffset, 
                      float revToMetersConversionFactor, 
                      float revToDegrees) {
    
    this.extendOffset = extendOffset;
    this.rotateOffset = rotateOffset;


    extendSparkMax = new CANSparkMax(extendSparkMaxId, MotorType.kBrushless);
    extendSparkMax.setSmartCurrentLimit(30);
    extendSparkMax.setSoftLimit(SoftLimitDirection.kForward,-95);

    rotateSparkMax = new CANSparkMax(rotateSparkMaxId, MotorType.kBrushless);
    rotateSparkMax.setSmartCurrentLimit(40);
    rotateSparkMax.setIdleMode(IdleMode.kCoast);
    rotateSparkMax.setInverted(true);

    rotateFollowSparkMax = new CANSparkMax(rotateSparkMaxFollowId, MotorType.kBrushless);
    rotateFollowSparkMax.setSmartCurrentLimit(40);
    rotateFollowSparkMax.follow(rotateSparkMax);

    extendEncoder = extendSparkMax.getEncoder();
    extendEncoder.setPositionConversionFactor(revToMetersConversionFactor);
    extendEncoder.setVelocityConversionFactor(revToMetersConversionFactor);
    extendEncoder.setPosition(-25.285551+4);

    rotatingEncoder = rotateSparkMax.getAlternateEncoder(AlternateEncoderType.kQuadrature,8192);
    rotatingEncoder.setPositionConversionFactor(revToDegrees);
    rotatingEncoder.setVelocityConversionFactor(revToDegrees);
    //rotatingEncoder.setZeroOffset(0);
    // updateRelativeEncoders();
    
    desiredDistance = 0;
    currentDistance = extendEncoder.getPosition();

    SparkMaxPIDController extendpid = extendSparkMax.getPIDController();
    SparkMaxPIDController rotatepid = rotateSparkMax.getPIDController();

    extendpid.setP(Arm.pExtension);
    extendpid.setD(Arm.dExtension);
    extendpid.setI(Arm.iExtension);

    rotatepid.setP(Arm.pRotating);
    rotatepid.setI(Arm.iRotating);
    rotatepid.setD(Arm.dRotating);

    // extendpid.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve,0);
    // extendpid.setSmartMotionMaxAccel(Arm.maxAccelExtend, 0);
    // extendpid.setSmartMotionMaxVelocity(Arm.maxSpeedExtend, 0);

    // rotatepid.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve,0);
    // rotatepid.setSmartMotionMaxAccel(Arm.maxAccelRotate, 0);
    // rotatepid.setSmartMotionMaxVelocity(Arm.maxSpeedRotate, 0);

    setDesiredDistance(-1);
    setDesiredRotation(0);

    // extensionPID = new PIDController(Arm.pExtension, Arm.iExtension, Arm.dExtension);
    // rotatingPID = new PIDController(Arm.pRotating, Arm.iRotating, Arm.dRotating);
  }

  @Override
  public void periodic() {
    // if (Math.abs(desiredHeight - currentHeight) > 0.02) { // want to change this to pid
    //   currentDistance = rotatingEncoder.getPosition();
    //   rotateSparkMax.set((desiredHeight - currentHeight) > 0 ? 0.3 : -0.3);
    //   SmartDashboard.putString("armState", "stoping"); // might want to change this to the built in one  
    // } else {
    //   rotateSparkMax.stopMotor();
    //   SmartDashboard.putString("armState", "stoping");
    // }

    // rotateSparkMax.getPIDController().setFF(0.1 * Math.sin(Math.toRadians(currentHeight)));

    // if (Math.abs(desiredHeight - currentHeight) > 1) {
    //   currentHeight = rotatingEncoder.getPosition();
    // }
    SmartDashboard.putNumber("arm location", extendEncoder.getPosition());
    SmartDashboard.putNumber("arm rotation", rotatingEncoder.getPosition());
    SmartDashboard.putNumber("arm dlocation",desiredDistance);
    SmartDashboard.putNumber("arm drotation", desiredHeight);
    SmartDashboard.putNumber("arm extension current", extendSparkMax.getOutputCurrent());

    double armBottomeExtension = SmartDashboard.getNumber("arm bottom extension",-51);
    double armBottomRotation = SmartDashboard.getNumber("arm bottom rotation",34);
    double armMiddleExtension = SmartDashboard.getNumber("arm middle extension",-60.35);
    double armMiddleRotation = SmartDashboard.getNumber("arm middle rotation",88);
    double armUpperExtension = SmartDashboard.getNumber("arm upper extension", -85);
    double armUpperRotation = SmartDashboard.getNumber("arm upper rotation", 100.955515);
    // double armPickupRotation = SmartDashboard.getNumber("arm pickup rotation", 85.142151);
    double armPickupRotation = SmartDashboard.getNumber("arm pickup rotation", 85.14);

    double armPickupExtension = SmartDashboard.getNumber("arm pickup extension", 0);

    armPositions[Positions.LOW.ordinal()] = new ArmPosition(armBottomeExtension, armBottomRotation);
    armPositions[Positions.MIDDLE.ordinal()] = new ArmPosition(armMiddleExtension, armMiddleRotation);
    armPositions[Positions.UP.ordinal()] = new ArmPosition(armUpperExtension, armUpperRotation);
    armPositions[Positions.PICKUPSTATION.ordinal()] = new ArmPosition(armPickupExtension, armPickupRotation);

  }

  public void setDesiredDistance(double distance) {
    desiredDistance = distance;
    extendSparkMax.getPIDController().setReference(distance, ControlType.kPosition);
  }

  public void setDesiredRotation(double rotation) {
    desiredHeight = rotation;
    rotateSparkMax.getPIDController().setReference(rotation, ControlType.kPosition);
  }

  public void groundSetPoint() {
    setDesiredRotation(0); // zero down?
  }

  public void setPosition(Positions p) {
    ArmPosition pos = armPositions[p.ordinal()];
    setRawPosition(pos);
  } 

  public void setZeroPosition() {
    extendEncoder.setPosition(0.0);
  }

  public void setRawPosition(ArmPosition pos) {
    desiredDistance = pos.extension;
    desiredHeight = pos.rotation;
    setDesiredDistance(desiredDistance);
    setDesiredRotation(desiredHeight);
  }

  public void setPercentage(double amount) {
    rotateSparkMax.set(amount);
  }
  public void setPercentageex(double amount) {
    extendSparkMax.set(amount);
  }

  public double getExtension() {
    return extendEncoder.getPosition();
  }

  public double getRotation() {
    return rotatingEncoder.getPosition();
  }

  public boolean leftSide(Positions pos) {
    return getPostionAngle(pos) < 0;
  }

  public boolean leftSide(double angle) {
    return false;
  }

  public double getPostionAngle(Positions pos) {
    return armPositions[pos.ordinal()].rotation;
  }

  public double getPostionExtension(Positions pos) {
    return armPositions[pos.ordinal()].extension;
  }

  public void stopMotors() {
    rotateSparkMax.stopMotor();
  }

  public void bringIn() {
    setDesiredDistance(0);
  }

  

//   public void updateRelativeEncoders() {
//     extendEncoder.setPosition(extendCANCoder.getAbsolutePosition() - extendOffset);
//     rotatingEncoder.setPosition(rotateCANCoder.getAbsolutePosition() - rotateOffset);
//   }

  // public void extendToDistanceMeters(float distance) {
      
  // }
}
