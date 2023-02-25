// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.IntakeSubsystem;

// /**
//  * An example command that uses an example subsystem.
//  */
// public class StopIntake extends CommandBase
// {

//   private final IntakeSubsystem intakeSubsystem;

//   /**
//    * Extend arm to given distance in meters
//    * might not matter if it is a cone or a cube but... it's there if it is needed
//    */
//   public StopIntake(IntakeSubsystem i)
//   {
//     intakeSubsystem = i;
//     addRequirements(intakeSubsystem);
//   }

//   @Override
//   public void initialize() {
//     intakeSubsystem.setBottomMotorSpeed(0);
//     intakeSubsystem.setTopMotorSpeed(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished()
//   {
//     return true;
//   }
// }
