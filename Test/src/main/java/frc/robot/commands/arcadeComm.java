// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.arcadeSubsys;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class arcadeComm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final arcadeSubsys m_subsystem;
  private final DoubleSupplier forwardTrigger, backTrigger, turning;


  /**
   * Creates a new arcadeComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public arcadeComm(arcadeSubsys subsystem, DoubleSupplier forward, DoubleSupplier backward, DoubleSupplier turn) {
    m_subsystem = subsystem;
    forwardTrigger = forward;
    backTrigger = backward;
    turning = turn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.motionTurning(forwardTrigger.getAsDouble() - backTrigger.getAsDouble(), turning.getAsDouble());
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
