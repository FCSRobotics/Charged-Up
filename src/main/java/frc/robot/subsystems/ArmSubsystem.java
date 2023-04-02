// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.utils.ArmPosition;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class ArmSubsystem extends SubsystemBase
{

  public final CANSparkMax extendSparkMax;
  public final CANSparkMax rotateSparkMax;
  public final PIDController rotatePIDController;
  public final CANSparkMax rotateFollowSparkMax;
  public final RelativeEncoder extendEncoder;
  public final SparkMaxAbsoluteEncoder rotatingEncoder;
//   public final CANCoder extendCANCoder;
//   public final CANCoder rotateCANCoder;
  public double desiredDistance;
  

  private SparkMaxAbsoluteEncoder extendAbsoluteEncoder;

  //public DutyCycleEncoder absoluteEncoder;
  
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
    new ArmPosition(0.5, 93.142197), //high back
    new ArmPosition(0.5, 80.01), //mid back
    new ArmPosition(0.5, 34), //low back
    new ArmPosition(0, 79.142250), // change this to back
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
    extendSparkMax.restoreFactoryDefaults();
    extendSparkMax.setSmartCurrentLimit(40);
    // extendSparkMax.setSoftLimit(SoftLimitDirection.kForward,-95);
    

    rotateSparkMax = new CANSparkMax(rotateSparkMaxId, MotorType.kBrushless);
    rotateSparkMax.setSmartCurrentLimit(40);
    rotateSparkMax.setIdleMode(IdleMode.kCoast);
    rotateSparkMax.setInverted(false);

    rotateFollowSparkMax = new CANSparkMax(rotateSparkMaxFollowId, MotorType.kBrushless);
    rotateFollowSparkMax.setSmartCurrentLimit(40);
    rotateFollowSparkMax.follow(rotateSparkMax,true);
    
    extendEncoder = extendSparkMax.getEncoder();

    extendAbsoluteEncoder = extendSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    extendAbsoluteEncoder.setPositionConversionFactor(1);
    extendEncoder.setPositionConversionFactor(revToMetersConversionFactor);
    extendEncoder.setVelocityConversionFactor(revToMetersConversionFactor);
    
    extendEncoder.setPosition((extendAbsoluteEncoder.getPosition())-0.7);
    

    rotatingEncoder = rotateSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    rotatingEncoder.setPositionConversionFactor(revToDegrees);
    rotatingEncoder.setVelocityConversionFactor(revToDegrees);

    rotatingEncoder.setZeroOffset(157.674713);

    // absoluteEncoder = new DutyCycleEncoder(0);
    // absoluteEncoder.setDistancePerRotation(360);
    // absoluteEncoder.setPositionOffset(0);
    // rotatingEncoder.setPosition(absoluteEncoder.getDistance()-222);
    rotatePIDController = new PIDController(Arm.pRotating,Arm.iRotating,Arm.dRotating);
    rotatePIDController.enableContinuousInput(0, 360);
    // absoluteEncoder.close();
    
    desiredDistance = 0;
    currentDistance = -1 *extendEncoder.getPosition();

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

    setDesiredDistance(0);
    setDesiredRotation(0);
    SmartDashboard.putBoolean("Desired Rotation set", false);

    // extensionPID = new PIDController(Arm.pExtension, Arm.iExtension, Arm.dExtension);
    // rotatingPID = new PIDController(Arm.pRotating, Arm.iRotating, Arm.dRotating);
  }

  @Override
  public void periodic() {
    DriverStation.reportWarning("Desired distance: " + desiredDistance, false);
    //extendSparkMax.getPIDController().setReference(desiredDistance, ControlType.kPosition);

    double currentArmFeedforward = Arm.feedForwardMap[calculateIndexFromAngle((int)rotatingEncoder.getPosition())];

    double output = MathUtil.clamp(rotatePIDController.calculate(360 - rotatingEncoder.getPosition(),desiredHeight),-4.8,4.8);


    double voltageVal = output + currentArmFeedforward;
    SmartDashboard.putNumber("the amount of voltage being sent to the arm at this moment",voltageVal);
    rotateSparkMax.setVoltage(voltageVal );

    // rotateSparkMax.getPIDController().setFF(0);
    SmartDashboard.putNumber("arm output voltage", output+currentArmFeedforward);
    SmartDashboard.putNumber("arm feed forward",currentArmFeedforward);
    SmartDashboard.putNumber("arm index",calculateIndexFromAngle((int)rotatingEncoder.getPosition()));

    

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
    //DriverStation.reportWarning("encoder connected: " + absoluteEncoder.isConnected(), false);
    SmartDashboard.putNumber("arm location", -1 * extendEncoder.getPosition());
    SmartDashboard.putNumber("arm absolute location", extendAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("arm rotation", rotatingEncoder.getPosition());
    SmartDashboard.putNumber("arm extension conversion factor", extendEncoder.getPositionConversionFactor());
    
    SmartDashboard.putNumber("arm dlocation",desiredDistance);
    SmartDashboard.putNumber("arm drotation", desiredHeight);
    
    SmartDashboard.putNumber("arm extension current", extendSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("annoying rev encoder rotation", rotatingEncoder.getPosition());


    /*high score 360 - 260.616547 0.808414
middle score 360 - 274.635986 0.281239
intake 
player  279.909149 0 */

    double armBottomeExtension = 0;
    double armBottomRotation = 360 - 40;
    double armMiddleRotation =  360 - 274.635986;
    double armUpperExtension = 0.808414;
    double armUpperRotation = 360 - 260.616547 + 5;
    double armMiddleExtension = 0.281239;
    // double armPickupRotation = SmartDashboard.getNumber("arm pickup rotation", 85.142151);
    double armPickupRotation = 360 - 279.909149 - 3;

    double armPickupExtension = 0;

    // double armBottomeExtension = SmartDashboard.getNumber("arm bottom extension",0);
    // double armBottomRotation = SmartDashboard.getNumber("arm bottom rotation",75);
    // double armMiddleExtension = SmartDashboard.getNumber("arm middle extension",0);
    // double armMiddleRotation = SmartDashboard.getNumber("arm middle rotation",90);
    // double armUpperExtension = SmartDashboard.getNumber("arm upper extension",0);
    // double armUpperRotation = SmartDashboard.getNumber("arm upper rotation", 105);
    // // double armPickupRotation = SmartDashboard.getNumber("arm pickup rotation", 85.142151);
    // double armPickupRotation = SmartDashboard.getNumber("arm pickup rotation", 60);

    // double armPickupExtension = SmartDashboard.getNumber("arm pickup extension", 0);

    SmartDashboard.putNumber("arm rotation voltage: ", rotateSparkMax.getBusVoltage()*rotateSparkMax.get());
    

    armPositions[Positions.LOW.ordinal()] = new ArmPosition(armBottomeExtension, armBottomRotation);
    armPositions[Positions.MIDDLE.ordinal()] = new ArmPosition(armMiddleExtension, armMiddleRotation);
    armPositions[Positions.UP.ordinal()] = new ArmPosition(armUpperExtension, armUpperRotation);
    armPositions[Positions.PICKUPSTATION.ordinal()] = new ArmPosition(armPickupExtension, armPickupRotation);
  }

  public void setDesiredDistance(double distance) {
    DriverStation.reportWarning("Desired distance set" , false);
    desiredDistance = distance;
    extendSparkMax.getPIDController().setReference(-distance, ControlType.kPosition);
  }

  public void setDesiredRotation(double rotation) {
    desiredHeight = rotation;

    //rotateSparkMax.getPIDController().setReference(rotation, ControlType.kPosition);
  }

  public void groundSetPoint() {
    setDesiredRotation(0); // zero down?
  }

  public void setPosition(Positions p) {
    ArmPosition pos = armPositions[p.ordinal()];
    setRawPosition(pos);
  } 

  public void setZeroPosition() {
    // extendEncoder.setPosition(0.0);
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

  private int calculateIndexFromAngle(int angle) {
    int index = (int)Math.round(Math.floor(angle / 6));
    if (index < 0) {
      return 0;
    } else if (index > 5) {
      return 5;
    }
    return index;
  }
  
  public void resetAbsoluteEncoder() {
    rotatingEncoder.setZeroOffset(157.674713);
    extendEncoder.setPosition(extendAbsoluteEncoder.getPosition());
    
  }

  public void finalize() {
    
  }

  

//   public void updateRelativeEncoders() {
//     extendEncoder.setPosition(extendCANCoder.getAbsolutePosition() - extendOffset);
//     rotatingEncoder.setPosition(rotateCANCoder.getAbsolutePosition() - rotateOffset);
//   }

  // public void extendToDistanceMeters(float distance) {
      
  // }
}
