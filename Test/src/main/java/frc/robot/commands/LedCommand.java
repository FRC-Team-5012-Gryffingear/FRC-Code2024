// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LedCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final LedSubsystem ledsubsys;
 // private Timer time = new Timer();

  /**
   * Creates a new LedCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LedCommand(LedSubsystem subsystem) {
    ledsubsys = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // ledsubsys.LED_off(0);
    // ledsubsys.LED_off(1);
    // ledsubsys.LED_off(2);
    // ledsubsys.LED_off(3);
    // ledsubsys.LED_off(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ledsubsys.LED_on(4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
