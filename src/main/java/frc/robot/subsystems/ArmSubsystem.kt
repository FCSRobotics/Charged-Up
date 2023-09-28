// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.Arm
import frc.robot.utils.ArmPosition

class ArmSubsystem(extendSparkMaxId: Int,
                   rotateSparkMaxId: Int,
                   rotateSparkMaxFollowId: Int,
                   val extendOffset: Float,
                   val rotateOffset: Float,
                   revToMetersConversionFactor: Float,
                   revToDegrees: Float) : SubsystemBase() {
    val extendSparkMax: CANSparkMax
    val rotateSparkMax: CANSparkMax
    val rotatePIDController: PIDController
    val rotateFollowSparkMax: CANSparkMax
    val extendEncoder: RelativeEncoder
    val rotatingEncoder: SparkMaxAbsoluteEncoder

    //   public final CANCoder extendCANCoder;
    //   public final CANCoder rotateCANCoder;
    var desiredDistance: Double = 0.0
        public set(distance) {
            DriverStation.reportWarning("Desired distance set", false)
            field = distance
            extendSparkMax.pidController.setReference(-distance, CANSparkMax.ControlType.kPosition)
        }
    private val extendAbsoluteEncoder: SparkMaxAbsoluteEncoder

    //public DutyCycleEncoder absoluteEncoder;
    var currentDistance: Double
    var desiredHeight = 0.0

    var currentHeight = 0.0

    enum class Positions {
        UP,
        MIDDLE,
        LOW,
        PICKUPSTATION,
        PICKUP,
        MIDDLE_CUBE,
        HIGH_CUBE
    }

    val armPositions = arrayOf(
            ArmPosition(0.5, 93.142197),  //high back
            ArmPosition(0.7, 80.01),  //mid back
            ArmPosition(0.5, 34.0),  //low back
            ArmPosition(0.0, 79.142250),  // change this to back
            ArmPosition(0.0, 0.0))

    init {
        extendSparkMax = CANSparkMax(extendSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless)
        extendSparkMax.restoreFactoryDefaults()
        extendSparkMax.setSmartCurrentLimit(40)
        // extendSparkMax.setSoftLimit(SoftLimitDirection.kForward,-95);
        rotateSparkMax = CANSparkMax(rotateSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless)
        rotateSparkMax.restoreFactoryDefaults()
        rotateSparkMax.setSmartCurrentLimit(40)
        rotateSparkMax.setIdleMode(IdleMode.kCoast)
        rotateSparkMax.inverted = false
        rotateFollowSparkMax = CANSparkMax(rotateSparkMaxFollowId, CANSparkMaxLowLevel.MotorType.kBrushless)
        rotateFollowSparkMax.setSmartCurrentLimit(40)
        rotateFollowSparkMax.follow(rotateSparkMax, true)
        extendEncoder = extendSparkMax.encoder
        extendAbsoluteEncoder = extendSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        extendAbsoluteEncoder.setPositionConversionFactor(1.0) //20 * revToMetersConversionFactor);
        extendEncoder.setPositionConversionFactor(revToMetersConversionFactor.toDouble())
        extendEncoder.setVelocityConversionFactor(revToMetersConversionFactor.toDouble())
        extendEncoder.setPosition(extendAbsoluteEncoder.position - 0.7)
        // extendEncoder.setPosition((-extendAbsoluteEncoder.getPosition())-0.048719);
        rotatingEncoder = rotateSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        rotatingEncoder.setPositionConversionFactor(revToDegrees.toDouble())
        rotatingEncoder.setVelocityConversionFactor(revToDegrees.toDouble())
        rotatingEncoder.setZeroOffset(157.674713)

        // absoluteEncoder = new DutyCycleEncoder(0);
        // absoluteEncoder.setDistancePerRotation(360);
        // absoluteEncoder.setPositionOffset(0);
        // rotatingEncoder.setPosition(absoluteEncoder.getDistance()-222);
        rotatePIDController = PIDController(Arm.pRotating, Arm.iRotating, Arm.dRotating)
        rotatePIDController.enableContinuousInput(0.0, 360.0)
        // absoluteEncoder.close();
        desiredDistance = 0.0
        currentDistance = -1 * extendEncoder.position
        val extendpid = extendSparkMax.pidController
        val rotatepid = rotateSparkMax.pidController
        extendpid.setP(Arm.pExtension)
        extendpid.setD(Arm.dExtension)
        extendpid.setI(Arm.iExtension)
        rotatepid.setP(Arm.pRotating)
        rotatepid.setI(Arm.iRotating)
        rotatepid.setD(Arm.dRotating)


        // extendpid.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve,0);
        // extendpid.setSmartMotionMaxAccel(Arm.maxAccelExtend, 0);
        // extendpid.setSmartMotionMaxVelocity(Arm.maxSpeedExtend, 0);

        // rotatepid.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kSCurve,0);
        // rotatepid.setSmartMotionMaxAccel(Arm.maxAccelRotate, 0);
        // rotatepid.setSmartMotionMaxVelocity(Arm.maxSpeedRotate, 0);
        desiredDistance = 0.0
        desiredHeight = 0.0
        SmartDashboard.putBoolean("Desired Rotation set", false)

        // extensionPID = new PIDController(Arm.pExtension, Arm.iExtension, Arm.dExtension);
        // rotatingPID = new PIDController(Arm.pRotating, Arm.iRotating, Arm.dRotating);
    }

    override fun periodic() {
        DriverStation.reportWarning("Desired distance: $desiredDistance", false)
        //extendSparkMax.getPIDController().setReference(desiredDistance, ControlType.kPosition);
        val currentArmFeedforward = Arm.feedForwardMap[calculateIndexFromAngle(rotatingEncoder.position.toInt())]
        val output = MathUtil.clamp(rotatePIDController.calculate(360 - rotatingEncoder.position, desiredHeight), -4.8, 5.8)
        val voltageVal = output + currentArmFeedforward
        SmartDashboard.putNumber("the amount of voltage being sent to the arm at this moment", voltageVal)
        rotateSparkMax.setVoltage(voltageVal)

        // rotateSparkMax.getPIDController().setFF(0);
        SmartDashboard.putNumber("arm output voltage", output + currentArmFeedforward)
        SmartDashboard.putNumber("arm feed forward", currentArmFeedforward)
        SmartDashboard.putNumber("arm index", calculateIndexFromAngle(rotatingEncoder.position.toInt()).toDouble())


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
        SmartDashboard.putNumber("arm location", -1 * extendEncoder.position)
        SmartDashboard.putNumber("arm absolute location", extendAbsoluteEncoder.position)
        SmartDashboard.putNumber("arm rotation", rotatingEncoder.position)
        SmartDashboard.putNumber("arm extension conversion factor", extendEncoder.positionConversionFactor)
        SmartDashboard.putNumber("arm dlocation", desiredDistance)
        SmartDashboard.putNumber("arm drotation", desiredHeight)
        SmartDashboard.putNumber("arm extension current", extendSparkMax.outputCurrent)
        SmartDashboard.putNumber("annoying rev encoder rotation", rotatingEncoder.position)


        /*high score 360 - 260.616547 0.808414
middle score 360 - 274.635986 0.281239
intake 
player  279.909149 0 */
        val armBottomeExtension = 0.0
        val armBottomRotation = 0.0
        val armMiddleRotation = 360 - 270 - 2.5
        val armUpperExtension = 0.86
        val armUpperRotation = 360 - 260.616547 + 5
        val armMiddleExtension = 0.3
        // double armPickupRotation = SmartDashboard.getNumber("arm pickup rotation", 85.142151);
        val armPickupRotation = 81.840271
        val armPickupExtension = 0.0

        // double armBottomeExtension = SmartDashboard.getNumber("arm bottom extension",0);
        // double armBottomRotation = SmartDashboard.getNumber("arm bottom rotation",75);
        // double armMiddleExtension = SmartDashboard.getNumber("arm middle extension",0);
        // double armMiddleRotation = SmartDashboard.getNumber("arm middle rotation",90);
        // double armUpperExtension = SmartDashboard.getNumber("arm upper extension",0);
        // double armUpperRotation = SmartDashboard.getNumber("arm upper rotation", 105);
        // // double armPickupRotation = SmartDashboard.getNumber("arm pickup rotation", 85.142151);
        // double armPickupRotation = SmartDashboard.getNumber("arm pickup rotation", 60);

        // double armPickupExtension = SmartDashboard.getNumber("arm pickup extension", 0);
        SmartDashboard.putNumber("arm rotation voltage: ", rotateSparkMax.busVoltage * rotateSparkMax.get())
        armPositions[Positions.LOW.ordinal] = ArmPosition(armBottomeExtension, armBottomRotation)
        armPositions[Positions.MIDDLE.ordinal] = ArmPosition(armMiddleExtension, armMiddleRotation)
        armPositions[Positions.UP.ordinal] = ArmPosition(armUpperExtension, armUpperRotation)
        armPositions[Positions.PICKUPSTATION.ordinal] = ArmPosition(armPickupExtension, armPickupRotation)
    }





    fun groundSetPoint() {
        desiredHeight = 0.0 // zero down?
    }

    fun setPosition(p: Positions) {
        val pos = armPositions[p.ordinal]
        setRawPosition(pos)
    }

    fun setZeroPosition() {
        // extendEncoder.setPosition(0.0);
    }

    fun setRawPosition(pos: ArmPosition) {
        desiredDistance = pos.extension
        desiredHeight = pos.rotation
        desiredDistance = desiredDistance
        // useless line of code? setDesiredRotation(desiredHeight)
    }

    fun setPercentage(amount: Double) {
        rotateSparkMax.set(amount)
    }

    fun setPercentageex(amount: Double) {
        extendSparkMax.set(amount)
    }

    val extension: Double
        get() = extendEncoder.position
    val rotation: Double
        get() = rotatingEncoder.position

    fun leftSide(pos: Positions): Boolean {
        return getPostionAngle(pos) < 0
    }

    fun leftSide(angle: Double): Boolean {
        return false
    }

    fun getPostionAngle(pos: Positions): Double {
        return armPositions[pos.ordinal].rotation
    }

    fun getPostionExtension(pos: Positions): Double {
        return armPositions[pos.ordinal].extension
    }

    fun stopMotors() {
        rotateSparkMax.stopMotor()
    }

    fun bringIn() {
        desiredDistance = 0.0
    }

    private fun calculateIndexFromAngle(angle: Int): Int {
        val index = Math.round(Math.floor((angle / 6).toDouble())).toInt()
        if (index < 0) {
            return 0
        } else if (index > 5) {
            return 5
        }
        return index
    }

    fun resetAbsoluteEncoder() {
        rotatingEncoder.setZeroOffset(157.674713)
        extendEncoder.setPosition(extendAbsoluteEncoder.position)
    }

    fun finalize() {} //   public void updateRelativeEncoders() {
    //     extendEncoder.setPosition(extendCANCoder.getAbsolutePosition() - extendOffset);
    //     rotatingEncoder.setPosition(rotateCANCoder.getAbsolutePosition() - rotateOffset);
    //   }
    // public void extendToDistanceMeters(float distance) {
    // }
}
