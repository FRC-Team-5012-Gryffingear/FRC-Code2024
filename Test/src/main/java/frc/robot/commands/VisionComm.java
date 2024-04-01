// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSub;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionComm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final VisionSub visionSubsystem;

  /**
   * Creates a new VisionComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionComm(VisionSub VisSubsystem) {
    visionSubsystem = VisSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(VisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WE ARE IN");
    visionSubsystem.startThread();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionSubsystem.startThread();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSubsystem.stopThread();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}