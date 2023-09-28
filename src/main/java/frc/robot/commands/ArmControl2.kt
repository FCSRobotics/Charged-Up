// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.Arm
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ArmSubsystem.Positions
import frc.robot.subsystems.GrabberSubsystem
import frc.robot.utils.MovingAverage
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import java.util.function.IntSupplier
import kotlin.math.abs

/**
 * An example command that uses an example subsystem.
 */
class ArmControl2(private val armSubsystem: ArmSubsystem, private val voltage: DoubleSupplier, private val extensionSupplier: DoubleSupplier, private val positionSupplier: IntSupplier,
                  private val intakeUp: BooleanSupplier, private val returnToZero: BooleanSupplier, private val grabberSubsystem: GrabberSubsystem) : CommandBase() {
    private var lastTimePositionHeld: Long = 0
    private var lastFrameAngle: Int
    private var lastInputedAngle: Int
    private var rotationMovingAverage: MovingAverage
    private val currentArmFeedforward: ArmFeedforward? = null

    /**
     * Extend arm to given distance in meters
     *
     * @param armSubsystem      The arm subsystem
     * @param distanceMeters    Target distance in meters
     *
     * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
     */
    init {
        lastFrameAngle = -1
        lastInputedAngle = -1
        rotationMovingAverage = MovingAverage(Arm.rollingAverageLength)


        // positions = new double[Arm.rollingAverageLength];
        // for (int i = 0; i < Arm.rollingAverageLength; i++) {
        //   positions[i] = 0;
        // }
        // currentSum = 0;
        lastTimePositionHeld = System.currentTimeMillis()
        addRequirements(armSubsystem)
    }

    override fun initialize() {
        SmartDashboard.putBoolean("check", false)
        grabberSubsystem.clamp()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        // armSubsystem.setPercentageex(extensionSupplier.getAsDouble());
        // currentArmFeedforward = Arm.feedForwardMap[calculateIndexFromAngle(positionSupplier.getAsInt())];
        val angle = positionSupplier.asInt
        if (angle != lastFrameAngle && lastFrameAngle == -1) {
            lastInputedAngle = if (angle == lastInputedAngle) {
                -1
            } else {
                angle
            }
        }
        lastFrameAngle = angle
        SmartDashboard.putNumber("last inputted angle", lastInputedAngle.toDouble())

        // if(lastInputedAngle != -1) {
        //   armSubsystem.setDesiredRotation(90);
        //   armSubsystem.setDesiredDistance(0.3);
        // } else {
        //   armSubsystem.setDesiredDistance(0);
        //   armSubsystem.setDesiredRotation(0);
        //   // Commands.sequence(new InstantCommand(() -> armSubsystem.setDesiredRotation(0), armSubsystem),new InstantCommand(() -> armSubsystem.setDesiredDistance(0), armSubsystem));
        // }

        //double currentAverage = currentSum/Arm.rollingAverageLength;
        val currentAverage = 360 - rotationMovingAverage.addValue(armSubsystem.rotation)
        SmartDashboard.putNumber("current average of arm position", currentAverage)
        val index = lastInputedAngle / 90
        val deltaAngle = voltage.asDouble * 20
        val deltaExtension = extensionSupplier.asDouble * 20
        if (lastInputedAngle == -1) {
            val currentTime = System.currentTimeMillis()
            if (!intakeUp.asBoolean && !returnToZero.asBoolean) {
                if (armSubsystem.extension >= -0.1) {
                    armSubsystem.desiredHeight = 0.0
                    if (Math.abs(armSubsystem.rotation) < 1) {
                        armSubsystem.stopMotors()
                    }
                } else {
                    lastTimePositionHeld = currentTime
                    armSubsystem.desiredDistance = 0.0
                    grabberSubsystem.clamp()
                }
            } else {
                if (armSubsystem.extension >= -0.1) {
                    armSubsystem.desiredHeight = 0.0
                    if (abs(armSubsystem.rotation) < 1) {
                        armSubsystem.stopMotors()
                    }
                }
                armSubsystem.desiredDistance = 0.0
                grabberSubsystem.clamp()
                lastTimePositionHeld = currentTime
            }
        } else {
            // armSubsystem.setDesiredDistance(0);
            lastTimePositionHeld = System.currentTimeMillis()
            if (armSubsystem.extension >= -0.1) {
                armSubsystem.desiredHeight = (armSubsystem.getPostionAngle(Positions.entries[index]) + deltaAngle)
                // DriverStation.reportWarning("oh shoot2");
            } else if (armSubsystem.leftSide(Positions.entries[index]) != armSubsystem.leftSide(armSubsystem.getPostionAngle(Positions.entries[index]) + deltaAngle)) {
                armSubsystem.desiredDistance = 0.0
            }
            if (abs(currentAverage - armSubsystem.getPostionAngle(Positions.entries[index]) - deltaAngle) < 20) {
                SmartDashboard.putBoolean("extension set", true)
                armSubsystem.desiredDistance = (armSubsystem.getPostionExtension(Positions.entries[index]))
                armSubsystem.desiredHeight = (armSubsystem.getPostionAngle(Positions.entries[index]) + deltaAngle)
            }
            // DriverStation.reportWarning("oh shoot");
        }

        //armSubsystem.setRawPosition(new ArmPosition(0, voltage.getAsDouble()));
        SmartDashboard.putNumber("Angle ", deltaAngle)
        SmartDashboard.putBoolean("check", true)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
