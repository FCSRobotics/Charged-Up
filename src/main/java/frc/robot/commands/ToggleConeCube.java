package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ToggleConeCube extends CommandBase
{

  private final LightSubsystem lightSubsystem;

  /**
   * Extend arm to given distance in meters
   * might not matter if it is a cone or a cube but... it's there if it is needed
   */
  public ToggleConeCube(LightSubsystem l){
  
    lightSubsystem = l;
    addRequirements(lightSubsystem);
  }

  @Override
  public void initialize() {
    lightSubsystem.toggleConeCube();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}