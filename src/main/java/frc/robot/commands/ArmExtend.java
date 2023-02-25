// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ArmSubsystem;


// /**
//  * An example command that uses an example subsystem.
//  */
// public class ArmExtend extends CommandBase
// {

//   private final ArmSubsystem armSubsystem;
//   private double distanceMeters;
  

//   /**
//    * Extend arm to given distance in meters
//    *
//    * @param armSubsystem      The arm subsystem
//    * @param distanceMeters    Target distance in meters
//    *                         
//    * @param acceptableDistanceMeters How close to the target position the final position of the arm must be in meters.
//    *                          
//    * 
//    */
//   public ArmExtend(ArmSubsystem armSubsystem, float distancMeters)
//   {
//     this.armSubsystem = armSubsystem;
//     this.distanceMeters = distancMeters;
    
//     addRequirements(armSubsystem);
//   }

//   @Override
//   public void initialize()
//   {
    
//     armSubsystem.setDesiredDistance(distanceMeters);
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   // @Override
//   // public void execute()
//   // {
//   //   this.currentDistance = this.armSubsystem.extendEncoder.getPosition();
//   //   this.armSubsystem.extendSparkMax.set(this.distanceMeters - this.currentDistance);
//   // }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted)
//   {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished()
//   {
//     return true;
//   }


// }
