// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix.sensors.CANCoder
import com.ctre.phoenix.sensors.Pigeon2
import com.pathplanner.lib.server.PathPlannerServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import swervelib.parser.SwerveParser
import java.io.File
import java.io.IOException

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
class Robot : TimedRobot() {
    private var m_autonomousCommand: Command? = null
    private var m_robotContainer: RobotContainer? = null
    private var disabledTimer: Timer? = null
    var frontleftCanCoder = CANCoder(33)
    var backleftCanCoder = CANCoder(35)
    var frontrightCanCoder = CANCoder(32)
    var backrightCanCoder = CANCoder(34)
    var pigeon = Pigeon2(31)

    init {
        instance = this
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    override fun robotInit() {
        PathPlannerServer.startServer(5811)
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = RobotContainer()

        // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
        // immediately when disabled, but then also let it be pushed more 
        disabledTimer = Timer()
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        SmartDashboard.putNumber("front left", frontleftCanCoder.absolutePosition)
        SmartDashboard.putNumber("front right", frontrightCanCoder.absolutePosition)
        SmartDashboard.putNumber("back left", backleftCanCoder.absolutePosition)
        SmartDashboard.putNumber("back right", backrightCanCoder.absolutePosition)
        SmartDashboard.putNumber("pigeon", pigeon.yaw)
        val table = NetworkTableInstance.getDefault().getTable("limelight")
        val ta = table.getEntry("ta").getDouble(0.0)
        SmartDashboard.putNumber("lime area", ta)
        val id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(DoubleArray(6))
        SmartDashboard.putNumberArray("april id", id)
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        m_robotContainer!!.periodic()
        CommandScheduler.getInstance().run()
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    override fun disabledInit() {
        m_robotContainer!!.setMotorBrake(true)
        disabledTimer!!.reset()
        disabledTimer!!.start()
    }

    override fun disabledPeriodic() {
        if (disabledTimer!!.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME)) {
            m_robotContainer!!.setMotorBrake(false)
            disabledTimer!!.stop()
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your [RobotContainer] class.
     */
    override fun autonomousInit() {
        m_robotContainer!!.setMotorBrake(true)
        m_autonomousCommand = m_robotContainer!!.autonomousCommand

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand!!.schedule()
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    override fun autonomousPeriodic() {
        m_robotContainer!!.autoPeriodic()
    }

    override fun teleopInit() {
        m_robotContainer!!.teleopInit()
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand!!.cancel()
        }
        m_robotContainer!!.setDriveMode()
        m_robotContainer!!.setMotorBrake(true)
    }

    /**
     * This function is called periodically during operator control.
     */
    override fun teleopPeriodic() {}
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
        try {
            SwerveParser(File(Filesystem.getDeployDirectory(), "swerve"))
        } catch (e: IOException) {
            throw RuntimeException(e)
        }
    }

    /**
     * This function is called periodically during test mode.
     */
    override fun testPeriodic() {}

    /**
     * This function is called once when the robot is first started up.
     */
    override fun simulationInit() {}

    /**
     * This function is called periodically whilst in simulation.
     */
    override fun simulationPeriodic() {}

    companion object {
        var instance: Robot? = null
            private set
    }
}
