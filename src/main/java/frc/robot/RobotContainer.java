// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Grabber;
import frc.robot.Constants.Intake;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopIntake;
import frc.robot.commands.ToggleIntakePosition;
import frc.robot.commands.swervedrive2.auto.Autos;
import frc.robot.commands.swervedrive2.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;

import java.io.File;

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
  final ArmSubsystem arm = new ArmSubsystem(Arm.extendSparkMaxId, Arm.revToMetersConversionFactor);
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandPS4Controller driverController = new CommandPS4Controller(0);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  PS4Controller driverXbox = new PS4Controller(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> (Math.abs(driverXbox.getLeftY()/.7) >
                                                                 OperatorConstants.LEFT_Y_DEADBAND)
                                                                ? driverXbox.getLeftY()/.7 : 0,
                                                          () -> (Math.abs(driverXbox.getLeftX()) >
                                                                 OperatorConstants.LEFT_X_DEADBAND)
                                                                ? driverXbox.getLeftX()/.7 : 0,
                                                          () -> -driverXbox.getRightX()/.7,
                                                          () -> -driverXbox.getRightY()/.7,
                                                          false);
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverController.getLeftY()) > OperatorConstants.LEFT_Y_DEADBAND) ? driverController.getLeftY() : 0,
        () -> (Math.abs(driverController.getLeftX()) > OperatorConstants.LEFT_X_DEADBAND) ? driverController.getLeftX() : 0,
        () -> -driverController.getRawAxis(3), () -> true, false);

    drivebase.setDefaultCommand(closedFieldRel);
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

    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 2).onTrue((new ToggleIntakePosition(intake)));
    new JoystickButton(driverXbox, 3).onTrue((new StartIntake(intake, false)))
                                                   .onFalse(new StopIntake(intake)); // no idea what button this is
//    new JoystickButton(driverXbox, 3).whileTrue(new InstantCommand(drivebase::lock, drivebase));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Autos.driveAndSpin(drivebase);
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
