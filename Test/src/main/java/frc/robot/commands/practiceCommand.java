// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.practiceArcade;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class practiceCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final practiceArcade m_subsystem;
  private final DoubleSupplier ftrigger, btrigger, turn;
  /**
   * Creates a new practiceCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public practiceCommand(practiceArcade subsystem, DoubleSupplier forward, DoubleSupplier backward, DoubleSupplier turning) {
    m_subsystem = subsystem;
    ftrigger = forward;
    btrigger = backward;
    turn = turning;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.move_and_turn(ftrigger.getAsDouble() - btrigger.getAsDouble() , turn.getAsDouble());
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
