// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.arcade;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.text.AbstractDocument.LeafElement;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class arcadeComm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final arcade arcade2;
  private final DoubleSupplier RT, LT, Turn;

  /**
   * Creates a new arcadeComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public arcadeComm(arcade subsystem, DoubleSupplier Le, DoubleSupplier Ri, DoubleSupplier Tu) {
    arcade2 = subsystem;
    LT = Le;
    RT = Ri;
    Turn = Tu;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arcade2.moveandturn(RT.getAsDouble() - LT.getAsDouble(), Turn.getAsDouble());
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
