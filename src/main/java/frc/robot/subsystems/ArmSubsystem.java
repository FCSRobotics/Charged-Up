// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.CAN;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.Arm;
// import frc.robot.utils.ArmPosition;

// import com.ctre.phoenix.sensors.CANCoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANPIDController.AccelStrategy;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// public class ArmSubsystem extends SubsystemBase
// {

//   public final CANSparkMax extendSparkMax;
//   public final CANSparkMax rotateSparkMax;
//   public final RelativeEncoder extendEncoder;
//   public final RelativeEncoder rotatingEncoder;
//   public final CANCoder extendCANCoder;
//   public final CANCoder rotateCANCoder;
//   public double desiredDistance;
//   public double currentDistance;
//   public double desiredHeight;
//   public double currentHeight;
//   public final float extendOffset;
//   public final float rotateOffset;

//   public enum Positions {
//     DOWN,
//     LOW_CONE,
//     MIDDLE_CONE,
//     HIGH_CONE,
//     LOW_CUBE,
//     MIDDLE_CUBE,
//     HIGH_CUBE
//   };

//   public final ArmPosition[] armPositions = {
//     new ArmPosition(0, 0), // down
//     new ArmPosition(0.1, 10), // low cone
//     new ArmPosition(0, 0), // middle cone 
//     new ArmPosition(0, 0), // high cone
//     new ArmPosition(0,0), // low cube
//     new ArmPosition(0,0), // middle cube
//     new ArmPosition(0, 0) // high cube
//   };

//   public ArmSubsystem(int extendSparkMaxId, 
//                       int rotateSparkMaxId, 
//                       int extendCancoderid,
//                       int rotateCancoderid,
//                       float extendOffset,
//                       float rotateOffset, 
//                       float revToMetersConversionFactor, 
//                       float revToDegrees) {
    
//     this.extendOffset = extendOffset;
//     this.rotateOffset = rotateOffset;

//     extendSparkMax = new CANSparkMax(extendSparkMaxId, MotorType.kBrushless);
//     rotateSparkMax = new CANSparkMax(rotateSparkMaxId, MotorType.kBrushless);
    
//     extendCANCoder = new CANCoder(extendCancoderid);
//     rotateCANCoder = new CANCoder(rotateCancoderid);

//     extendEncoder = extendSparkMax.getEncoder();
//     extendEncoder.setPositionConversionFactor(revToMetersConversionFactor);
//     extendEncoder.setVelocityConversionFactor(revToMetersConversionFactor);

//     rotatingEncoder = rotateSparkMax.getEncoder();
//     rotatingEncoder.setPositionConversionFactor(revToDegrees);
//     rotatingEncoder.setVelocityConversionFactor(revToDegrees);

//     updateRelativeEncoders();
    
//     desiredDistance = 0;
//     currentDistance = extendEncoder.getPosition();

//     SparkMaxPIDController extendpid = extendSparkMax.getPIDController();
//     SparkMaxPIDController rotatepid = rotateSparkMax.getPIDController();

//     extendpid.setP(Arm.pExtension);
//     extendpid.setD(Arm.dExtension);
//     extendpid.setI(Arm.iExtension);

//     rotatepid.setP(Arm.pRotating);
//     rotatepid.setI(Arm.iRotating);
//     rotatepid.setD(Arm.dRotating);

//     extendpid.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve,0);
//     extendpid.setSmartMotionMaxAccel(Arm.maxAccelExtend, 0);
//     extendpid.setSmartMotionMaxVelocity(Arm.maxSpeedExtend, 0);

//     rotatepid.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve,0);
//     rotatepid.setSmartMotionMaxAccel(Arm.maxAccelRotate, 0);
//     rotatepid.setSmartMotionMaxVelocity(Arm.maxSpeedRotate, 0);

//     // extensionPID = new PIDController(Arm.pExtension, Arm.iExtension, Arm.dExtension);
//     // rotatingPID = new PIDController(Arm.pRotating, Arm.iRotating, Arm.dRotating);
//   }

//   @Override
//   public void periodic() {

    

//     // if (Math.abs(desiredDistance - currentDistance) > 0.02) {
//     //   currentDistance = extendEncoder.getPosition();
//     //   extendSparkMax.set(extensionPID.calculate(currentDistance, desiredDistance)); // might want to change this to the built in one  
//     // }
//     // if (Math.abs(desiredHeight - currentHeight) > 1) {
//     //   currentHeight = rotatingEncoder.getPosition();
//     // }
//   }

//   public void setDesiredDistance(double distance) {
//     desiredDistance = distance;
//     extendSparkMax.getPIDController().setReference(distance, ControlType.kPosition);
//   }

//   public void setDesiredRotation(double rotation) {
//     desiredHeight = rotation;
//     rotateSparkMax.getPIDController().setReference(rotation, ControlType.kPosition);
//   }

//   public void groundSetPoint() {
//     setDesiredRotation(0); // zero down?
//   }

//   public void setPosition(Positions p) {
//     ArmPosition pos = armPositions[p.ordinal()];
//     desiredDistance = pos.extension;
//     desiredHeight = pos.rotation;
//   } 

//   public void updateRelativeEncoders() {
//     extendEncoder.setPosition(extendCANCoder.getAbsolutePosition() - extendOffset);
//     rotatingEncoder.setPosition(rotateCANCoder.getAbsolutePosition() - rotateOffset);
//   }

//   // public void extendToDistanceMeters(float distance) {
      
//   // }
// }
