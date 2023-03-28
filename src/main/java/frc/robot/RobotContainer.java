// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Grabber;
import frc.robot.Constants.Intake;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ArmControl2;
import frc.robot.commands.CloseGrabber;
import frc.robot.commands.CycleColor;
import frc.robot.commands.GrabberMotorsControl;
import frc.robot.commands.IntakeMotorsControl;
import frc.robot.commands.OpenGrabber;
import frc.robot.commands.PidBalance;
import frc.robot.commands.Scoring;
import frc.robot.commands.StartGrabberMotors;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopGrabberMotors;
import frc.robot.commands.StopIntake;

import frc.robot.commands.ToggleGrabberMotors;
import frc.robot.commands.ToggleIntakeMotors;
import frc.robot.commands.ToggleIntakePosition;
// import frc.robot.commands.SetIntakePosition;
// import frc.robot.commands.StartIntake;
// import frc.robot.commands.StopIntake;
// import frc.robot.commands.ToggleIntakePosition;
import frc.robot.commands.swervedrive2.auto.Autos;
import frc.robot.commands.swervedrive2.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive2;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
// import frc.robot.subsystems.LightSubsystem;
// import frc.robot.subsystems.GrabberSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;

import java.io.File;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  final IntakeSubsystem intake = new IntakeSubsystem(Intake.topMotor, 
                                                     Intake.bottomMotor, 
                                                     Intake.extendChannel, 
                                                     Intake.retractChannel, 
                                                     Intake.reverseSolenoid);
  final GrabberSubsystem grabber = new GrabberSubsystem(Grabber.rightMotorId,
                                                        Grabber.leftMotorId,
                                                        Grabber.extendChannel, 
                                                        Grabber.retractChannel);
  final ArmSubsystem arm = new ArmSubsystem(Arm.extendSparkMaxId,
                                            Arm.rotateSparkMaxId,
                                            Arm.rotateFollowSparkMaxId,
                                            // Arm.extendCancoderid,
                                            // Arm.rotateCancoderid,
                                            Arm.extendOffset,
                                            Arm.rotateOffset,
                                            Arm.revToMetersConversionFactor, 
                                            Arm.revToAngleConversionFactor);
  final LightSubsystem light = new LightSubsystem();
  final Pigeon2 gyro = new Pigeon2(31);
  

  // // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandPS4Controller driverController = new CommandPS4Controller(0);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  PS4Controller driverXbox = new PS4Controller(0);

  Joystick rightstick = new Joystick(2);
  Joystick leftstick = new Joystick(1);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private boolean l1buttonPressed = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
    

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
    SmartDashboard.putNumber("High rightx", driverXbox.getRightX());
    TeleopDrive2 closedFieldRel = new TeleopDrive2(
        drivebase,
        () -> (Math.abs(leftstick.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? (!leftstick.getRawButton(1) ? 1: .5) * leftstick.getY() : 0,
        () -> (Math.abs(leftstick.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? (!leftstick.getRawButton(1) ? 1 :  .5) * leftstick.getX() : 0,
        () -> rightstick.getX(), () -> true, false);

    // ArmControl armControl = new ArmControl(arm,() -> 
    //   Math.atan2(driverController.getLeftX(),  
    //   driverController.getLeftY()) * 180/Math.PI);

    ArmControl2 armControl = new ArmControl2(arm,
      () -> driverController.getLeftY(),
      () -> driverController.getRightY(), 
      () -> driverController.getHID().getPOV(),
      () -> intake.isIn(),
      () -> driverXbox.getL1Button(),
      grabber);

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

    drivebase.setDefaultCommand(new ParallelCommandGroup(closedFieldRel,armControl));
    m_chooser.setDefaultOption("leave and balance", Autos.leaveandbalance(drivebase, intake, grabber, arm));
    m_chooser.addOption("balance", Autos.setActionsBalance(drivebase, intake, arm, grabber));
    m_chooser.addOption("balance gyro", Autos.gyroBalance(drivebase, intake, arm, grabber, gyro));
    m_chooser.addOption("balance pid", Autos.pidbalance(drivebase, intake, arm, grabber, gyro));
    m_chooser.addOption("do nothing", Autos.nullAuto());
    m_chooser.addOption("spin", Autos.driveAndSpin(drivebase));
    m_chooser.addOption("just leave", Autos.leaveTheStadium(drivebase,arm,grabber));
    m_chooser.addOption("pickup from left", Autos.pickUpConeCube(drivebase, arm, grabber, intake, true));
    m_chooser.addOption("pickup from right", Autos.pickUpConeCube(drivebase, arm, grabber, intake, false));
    m_chooser.addOption("drop cone",Commands.sequence(Autos.dropOffCone(drivebase, arm, grabber)));
    m_chooser.addOption("example auto",Autos.exampleAuto(drivebase));
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
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
    






    new GrabberMotorsControl(grabber, () -> driverXbox.getRawAxis(3), () -> driverXbox.getRawButton(6));
    new IntakeMotorsControl(intake,() -> driverXbox.getRawAxis(2), () -> driverXbox.getRawButton(5));
    
    new JoystickButton(driverXbox, 4).onTrue(new ToggleIntakePosition(intake));

    new JoystickButton(driverXbox, 7).onTrue((new CloseGrabber(grabber)));
    new JoystickButton(driverXbox, 8).onTrue((new OpenGrabber(grabber)));

    new JoystickButton(driverXbox, 10).onTrue(new InstantCommand(arm::setZeroPosition));
    new JoystickButton(driverXbox,9).onTrue((new InstantCommand(light::cycleColor, light)));

    new JoystickButton(driverXbox, 3).onTrue(Scoring.thirdLevelCube(intake));
    new JoystickButton(driverXbox, 1).onTrue(Scoring.secondLevelCube(intake));
    new JoystickButton(driverXbox, 2).onTrue(Scoring.shootCube(intake));






    new JoystickButton(rightstick, 7).onTrue((new InstantCommand(drivebase::zeroGyro)));

    

    //new JoystickButton(driverXbox, 5).onTrue(new InstantCommand(light::cycleColor, light));

    new JoystickButton(leftstick, 1).onTrue(new ToggleIntakePosition(intake));
    new JoystickButton(leftstick,1).whileTrue(new StartIntake(intake,true,true));
    new JoystickButton(leftstick,1).onFalse(Commands.sequence(new ToggleIntakePosition(intake),new StopIntake(intake)));
    new JoystickButton(rightstick,5).onTrue(new StartIntake(intake,true,true));
    new JoystickButton(rightstick,5).onFalse(new StopIntake(intake));
    new JoystickButton(rightstick,6).onTrue((new StartIntake(intake,true,false)));
    new JoystickButton(rightstick,6).onFalse((new StopIntake(intake)));
    
    
    
    
    new JoystickButton(leftstick, 2).onTrue(Scoring.shootCube(intake));
    new JoystickButton(leftstick,7).onTrue(new InstantCommand(arm::resetAbsoluteEncoder));
    new JoystickButton(rightstick, 1).whileTrue(Scoring.playerStation(drivebase, intake));
    new JoystickButton(rightstick, 1).onFalse(Commands.sequence(new StopIntake(intake), new ToggleIntakePosition(intake)));
    new JoystickButton(rightstick,2).whileTrue(new PidBalance(drivebase,gyro,intake));

    
//     new JoystickButton(driverXbox, 3).onTrue((new StartIntake(intake, false)))
//                                                    .onFalse(new StopIntake(intake)); // no idea what button this is
// //    new JoystickButton(driverXbox, 3).whileTrue(new InstantCommand(drivebase::lock, drivebase));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    //light.playAuto();
    DriverStation.reportWarning(m_chooser.getSelected().toString(), false);
    // An example command will be run in autonomous
    return m_chooser.getSelected(); // Autos.leaveandbalance(drivebase,intake);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
