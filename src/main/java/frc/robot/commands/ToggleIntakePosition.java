// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.IntakeSubsystem;

// /**
//  * An example command that uses an example subsystem.
//  */
// public class ToggleIntakePosition extends CommandBase {

//   private final IntakeSubsystem intakeSubsystem;  

//   /**
//    * bring the intake in or out
//    * */
//   public ToggleIntakePosition(IntakeSubsystem intake) {
//     intakeSubsystem = intake;
//     addRequirements(intakeSubsystem);
//   }

//   @Override
//   public void initialize() {
//     intakeSubsystem.togglePosition();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
