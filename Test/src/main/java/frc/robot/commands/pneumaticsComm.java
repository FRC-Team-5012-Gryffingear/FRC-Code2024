// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.pneumaticsSubsys;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class pneumaticsComm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final pneumaticsSubsys m_subsystem;
  private final BooleanSupplier buttonA, buttonB;

  /**
   * Creates a new pneumaticsComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public pneumaticsComm(pneumaticsSubsys subsystem, BooleanSupplier a, BooleanSupplier b) {
    m_subsystem = subsystem;
    buttonA = a;
    buttonB = b;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.inAndOut(buttonA.getAsBoolean(), buttonB.getAsBoolean());
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
