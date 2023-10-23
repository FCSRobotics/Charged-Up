// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.button.JoystickButton

import frc.robot.commands.*
import frc.robot.commands.swervedrive2.auto.Autos
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive2
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ArmSubsystem.Positions
import frc.robot.subsystems.GrabberSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.LightSubsystem
import frc.robot.subsystems.swervedrive2.SwerveSubsystem
import java.io.File
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import java.util.function.IntSupplier
import kotlin.time.times

//import frc.robot.commands.CycleColor;
// import frc.robot.commands.SetIntakePosition;
// import frc.robot.commands.StartIntake;
// import frc.robot.commands.StopIntake;
// import frc.robot.commands.ToggleIntakePosition;
// import frc.robot.subsystems.LightSubsystem;
// import frc.robot.subsystems.GrabberSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the [Robot] periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    val drivebase = SwerveSubsystem(File(Filesystem.getDeployDirectory(), "swerve"))
    val intake = IntakeSubsystem(Constants.Intake.topMotor,
            Constants.Intake.bottomMotor,
            Constants.Intake.rotateMotor)
    val grabber = GrabberSubsystem(Constants.Grabber.rightMotorId,
            Constants.Grabber.leftMotorId,
            Constants.Grabber.extendChannel,
            Constants.Grabber.retractChannel)
    val arm = ArmSubsystem(Constants.Arm.extendSparkMaxId,
            Constants.Arm.rotateSparkMaxId,
            Constants.Arm.rotateFollowSparkMaxId,  // Arm.extendCancoderid,
            // Arm.rotateCancoderid,
            Constants.Arm.extendOffset,
            Constants.Arm.rotateOffset,
            Constants.Arm.revToMetersConversionFactor,
            Constants.Arm.revToAngleConversionFactor)
    val light = LightSubsystem()
    val gyro = Pigeon2(31)

    private enum class AutoLedBehavior {
        Dart,
        Alternate,

        // Ripple,
        // Blink,
        Rainbow
    }

    private val autoLedBehavior: AutoLedBehavior

    // // CommandJoystick rotationController = new CommandJoystick(1);
    // Replace with CommandPS4Controller or CommandJoystick if needed
    var driverController = CommandPS4Controller(0)

    // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
    var driverXbox = PS4Controller(0)
    var rightstick = Joystick(2)
    var leftstick = Joystick(1)
    private val m_chooser = SendableChooser<Command>()
    private val l1buttonPressed = false
    var zeroPitch = gyro.pitch
    var zeroRoll = gyro.roll
    var zeroYaw = gyro.yaw

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {


        // Configure the trigger bindings
        configureBindings()

//(Math.random() > 0.5) ? AutoLedBehavior.Rainbow : 
        autoLedBehavior = AutoLedBehavior.Dart;
        CameraServer.startAutomaticCapture()
        CameraServer.startAutomaticCapture()


        // AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
        //                                                       // Applies deadbands and inverts controls because joysticks
        //                                                       // are back-right positive while robot
        //                                                       // controls are front-left positive
        //                                                       () -> (Math.abs(leftstick.getY()) >
        //                                                              OperatorConstants.LEFT_Y_DEADBAND)
        //                                                             ?(!leftstick.getRawButton(1) ? 1: 0.5) * leftstick.getY() : 0,
        //                                                       () -> (Math.abs(leftstick.getX()) >
        //                                                              OperatorConstants.LEFT_X_DEADBAND)
        //                                                             ? (!leftstick.getRawButton(1) ? 1 :  0.5) * leftstick.getX() : 0,
        //                                                       () -> -rightstick.getX(),
        //                                                       () -> -rightstick.getY(),
        //                                                       false);
        SmartDashboard.putNumber("High rightx", driverXbox.rightX)
        val closedFieldRel = TeleopDrive2(
                drivebase,
                { if (Math.abs(leftstick.y) > Constants.OperatorConstants.LEFT_Y_DEADBAND) getMultiplier(leftstick) * leftstick.y else 0.0 },
                { if (Math.abs(leftstick.x) > Constants.OperatorConstants.LEFT_X_DEADBAND) getMultiplier(leftstick) * leftstick.x else 0.0 },
                { rightstick.x }, { true }, false)

        // ArmControl armControl = new ArmControl(arm,() -> 
        //   Math.atan2(driverController.getLeftX(),  
        //   driverController.getLeftY()) * 180/Math.PI);
        val armControl = ArmControl2(arm,
                { driverController.leftY * (if (driverXbox.getRawButton(6)) 0.2 else 1.0 )},
                { driverController.rightY },
                { driverController.hid.pov }, { intake.isIn },
                { driverXbox.l1Button },
                grabber)


        //GrabberMotorsControl grabberMotorsControl = new GrabberMotorsControl(grabber, () -> driverXbox.getRawAxis(3), () -> driverXbox.getRawButton(6));
        //IntakeMotorsControl intakeMotorsControl = new IntakeMotorsControl(intake,() -> driverXbox.getRawAxis(2), () -> driverXbox.getRawButton(5));

        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //   drivebase::getPose, // Pose2d supplier
        //   drivebase::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        //   drivebase.kinematics, // SwerveDriveKinematics
        //   new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        //   new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        //   driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
        //   eventMap,
        //   true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        //   drivebase // The drive subsystem. Used to properly set the requirements of path following commands
        // );

        drivebase.defaultCommand = ParallelCommandGroup(closedFieldRel, armControl) //, grabberMotorsControl, intakeMotorsControl));
        // m_chooser.setDefaultOption("leave and balance", Autos.leaveandbalance(drivebase, intake, grabber, arm,gyro));
        // m_chooser.addOption("balance", Autos.setActionsBalance(drivebase, intake, arm, grabber));
        // m_chooser.addOption("balance gyro", Autos.gyroBalance(drivebase, intake, arm, grabber, gyro));
        m_chooser.addOption("balance pid (score top level cone and balance)", Autos.pidbalance(drivebase, intake, arm, grabber, gyro))
        m_chooser.addOption("do nothing", Autos.nullAuto())
        // m_chooser.addOption("spin", Autos.driveAndSpin(drivebase));
        m_chooser.addOption("just leave (score top level cone and score mobility)", Autos.leaveTheStadium(drivebase, arm, grabber, intake))
        // m_chooser.addOption("pickup from left", Autos.pickUpConeCube(drivebase, arm, grabber, intake, true));
        // m_chooser.addOption("pickup from right", Autos.pickUpConeCube(drivebase, arm, grabber, intake, false));
        // m_chooser.addOption("third cone and balance", Autos.thirdAndBalance(drivebase, gyro, arm, grabber, intake));
        m_chooser.setDefaultOption("drop cone (score top level cone)", Autos.anotherRandomThing(drivebase, arm, grabber, intake))
        m_chooser.addOption("score and leave later", Commands.sequence(Autos.dropOffCone(drivebase, arm, grabber),
                WaitCommand((1000 + 2250 + 2000 - 250).toDouble()),
                SetIntakePosition(intake, true),
                MoveTime(drivebase, -1.0, 0.0, (1000 + 2250 + 2000 - 1250).toLong())))
        m_chooser.addOption("test auto", Autos.exampleAuto(drivebase))
        m_chooser.addOption("path auto", Autos.followPath(drivebase))
        // m_chooser.addOption("example auto",Autos.exampleAuto(drivebase));
        SmartDashboard.putData("the options for the autonomous period", m_chooser)
    }

    private fun getMultiplier(stick: Joystick): Double {
        return if (stick.getRawButton(1)) 0.5 else when (stick.pov) {
            0 -> 1.0
            180 -> 0.5
            else -> 0.82
        }
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger.Trigger] constructor with an arbitrary predicate, or via the
     * named factories in [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
     * [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        // new JoystickButton(driverXbox, 1).onTrue(new StartGrabberMotors(grabber,Grabber.motorSpeeds,true));
        // new JoystickButton(driverXbox, 1).onFalse(new StopGrabberMotors(grabber));

        // new JoystickButton(driverXbox,4).onTrue(new StartGrabberMotors(grabber,-Grabber.motorSpeeds,false));
        // new JoystickButton(driverXbox,4).onFalse(new StopGrabberMotors(grabber));


        // new JoystickButton(driverXbox, 2).onTrue((new StartIntake(intake,true,true)));
        // new JoystickButton(driverXbox, 2).onFalse((new StopIntake(intake)));

        // new JoystickButton(driverXbox, 3).onTrue((new StartIntake(intake,true,false)));
        // new JoystickButton(driverXbox,3).onFalse((new StopIntake(intake)));

        // new JoystickButton(driverXbox, 6).onTrue(new ToggleIntakePosition(intake));

        // new JoystickButton(rightstick, 7).onTrue((new InstantCommand(drivebase::zeroGyro)));

        // new JoystickButton(driverXbox, 7).onTrue((new CloseGrabber(grabber)));
        // new JoystickButton(driverXbox, 8).onTrue((new OpenGrabber(grabber)));

        // new JoystickButton(driverXbox, 10).onTrue(new InstantCommand(arm::setZeroPosition));

        // //new JoystickButton(driverXbox, 5).onTrue(new InstantCommand(light::cycleColor, light));

        // new JoystickButton(rightstick,1).onTrue(new ToggleIntakePosition(intake));
        // new JoystickButton(rightstick,5).onTrue(new StartIntake(intake,true,true));
        // new JoystickButton(rightstick,5).onFalse(new StopIntake(intake));
        // new JoystickButton(rightstick,6).onTrue((new StartIntake(intake,true,false)));
        // new JoystickButton(rightstick,6).onFalse((new StopIntake(intake)));


        // new JoystickButton(leftstick,8).onTrue((new InstantCommand(light::cycleColor, light)));
        // new JoystickButton(leftstick, 11).onTrue(Scoring.thirdLevelCube(intake));
        // new JoystickButton(leftstick, 12).onTrue(Scoring._thirdLevelCube(intake));
        // new JoystickButton(leftstick, 2).onTrue(Scoring.shootCube(intake));
        // new JoystickButton(leftstick,7).onTrue(new InstantCommand(arm::resetAbsoluteEncoder));
        // new JoystickButton(rightstick, 3).whileTrue(Scoring.playerStation(drivebase, intake));
        // new JoystickButton(rightstick, 3).onFalse(new StopIntake(intake));
        // new JoystickButton(leftstick,9).onTrue(new PidBalance(drivebase,gyro,intake));
        //JoystickButton(driverXbox, 4).onTrue(ToggleIntakePosition(intake))
        JoystickButton(driverXbox, 10).onTrue(CloseGrabber(grabber))
        JoystickButton(driverXbox, 9).onTrue(OpenGrabber(grabber))
        JoystickButton(driverXbox, 0).onTrue(InstantCommand({ arm.setZeroPosition() }))
        JoystickButton(driverXbox, 3).onTrue(Scoring.thirdLevelCube(intake))
        JoystickButton(driverXbox, 1).onTrue(Scoring.secondLevelCube(intake))
        JoystickButton(driverXbox, 2).onTrue(Scoring.shootCube(intake))
        JoystickButton(rightstick, 7).onTrue(InstantCommand({ drivebase.zeroGyro() }))


        //new JoystickButton(driverXbox, 5).onTrue(new InstantCommand(light::cycleColor, light));
        JoystickButton(rightstick, 1).onTrue(SetIntakePosition(intake, true))
        //new JoystickButton(rightstick,1).whileTrue(new StartIntake(intake,true,true));
        JoystickButton(rightstick, 1).onFalse(Commands.sequence(SetIntakePosition(intake, false), StopIntake(intake)))
        JoystickButton(rightstick, 5).onTrue(StartIntake(intake, true, true))
        JoystickButton(rightstick, 5).onFalse(StopIntake(intake))
        JoystickButton(rightstick, 6).onTrue(StartIntake(intake, true, false))
        JoystickButton(rightstick, 6).onFalse(StopIntake(intake))
        JoystickButton(leftstick, 8).onTrue(Commands.sequence(MoveArmPosition(arm, Positions.PICKUPSTATION), OpenGrabber(grabber)))
        JoystickButton(leftstick, 8).whileTrue(InstantCommand({ drivebase.drive(Translation2d(-0.5, 0.0), 0.0, true, false) }))
        JoystickButton(leftstick, 8).onFalse(CloseGrabber(grabber))
        JoystickButton(rightstick, 3).whileTrue(PidTurn(drivebase, gyro, 0.0))
        //JoystickButton(rightstick, 4).whileTrue(AutoScore.AllignMid(drivebase, gyro, arm, grabber))
        JoystickButton(leftstick, 2).onTrue(Scoring.shootCube(intake))
        JoystickButton(leftstick, 11).onTrue(InstantCommand({ intake.zeroMotors() }))
        JoystickButton(rightstick, 11).onTrue(InstantCommand({ grabber.zeroMotors() }))

        //new JoystickButton(leftstick,7).onTrue(new InstantCommand(arm::resetAbsoluteEncoder));
        JoystickButton(leftstick, 3).whileTrue(Scoring.playerStation(drivebase, intake))
        //JoystickButton(leftstick, 3).onFalse(Commands.sequence(StopIntake(intake), ToggleIntakePosition(intake)))
        //new JoystickButton(rightstick,2).whileTrue(new PidBalance(drivebase,gyro,intake));


//     new JoystickButton(driverXbox, 3).onTrue((new StartIntake(intake, false)))
//                                                    .onFalse(new StopIntake(intake)); // no idea what button this is
// //    new JoystickButton(driverXbox, 3).whileTrue(new InstantCommand(drivebase::lock, drivebase));
    }

    fun periodic() {
        grabber.setMotorsSpeeds(0.0, true)
        when (autoLedBehavior) {
            AutoLedBehavior.Dart -> light.dart()
            AutoLedBehavior.Alternate -> light.alternate()
            AutoLedBehavior.Rainbow -> light.rainbow()

        }
        when (DriverStation.getAlliance()) {
            Alliance.Red -> light.setColor(255, 0, 0, 0)
            Alliance.Blue -> light.setColor(0, 0, 255, 250)
            else -> light.setColor(0, 255, 0, 113)
        }
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() {
            //light.playAuto();
            DriverStation.reportWarning(m_chooser.selected.toString(), false)
            // An example command will be run in autonomous
            Shuffleboard.getTab("Game Tab").addString("Selected auto") { m_chooser.selected.toString() }
            return m_chooser.selected // Autos.leaveandbalance(drivebase,intake);
        }

    fun autoPeriodic() {}
    fun teleopInit() {
        //Random ran = new Random();

    }

    fun setDriveMode() {
        //drivebase.setDefaultCommand();
    }

    fun setMotorBrake(brake: Boolean) {
        drivebase.setMotorBrake(brake)
    }
}
