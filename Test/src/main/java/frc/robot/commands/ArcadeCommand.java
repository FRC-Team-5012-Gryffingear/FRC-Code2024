// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArcadeDriv;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArcadeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArcadeDriv arcadesubsys;
  private final DoubleSupplier forwardtrigger, backtrigger, turning;
  
  /**
   * Creates a new ArcadeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeCommand(ArcadeDriv subsystem, DoubleSupplier fTrigger, DoubleSupplier bTrigger, DoubleSupplier t) {
    arcadesubsys = subsystem;
    forwardtrigger = fTrigger;
    backtrigger = bTrigger;
    turning = t;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arcadesubsys.moveandturn(forwardtrigger.getAsDouble() - backtrigger.getAsDouble(), turning.getAsDouble());
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
