// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.pneumaticVacuum;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class vacuumComm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final pneumaticVacuum m_subsystem;
  private final BooleanSupplier right, left;

  /**
   * Creates a new vacuumComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public vacuumComm(pneumaticVacuum subsystem, BooleanSupplier r, BooleanSupplier l) {
    m_subsystem = subsystem;
    right = r;
    left = l;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.basicMovement(right.getAsBoolean(), left.getAsBoolean());
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
