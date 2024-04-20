// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.shooterSubsys;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class shooterComm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterSubsys m_subsystem;
  private final BooleanSupplier buttonA, buttonB;
  //private final 

  /**
   * Creates a new shooterComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public shooterComm(shooterSubsys subsystem, BooleanSupplier A, BooleanSupplier B) {
    m_subsystem = subsystem;
    buttonA = A;
    buttonB = B;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.shoot(buttonA.getAsBoolean(), buttonB.getAsBoolean());
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
