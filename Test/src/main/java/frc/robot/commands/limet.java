// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.limey;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class limet extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final limey limeSub;

  /**
   * Creates a new limet.
   *
   * @param subsystem The subsystem used by this command.
   */
  public limet(limey subsystem) {
    limeSub = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("AprilXValue", limeSub.getX());
    limeSub.estimate3DZ();
    SmartDashboard.putNumber("TZ Value", limeSub.getTZ());
    SmartDashboard.putNumber("Distance of ID " + limeSub.getId(), limeSub.estimate3DZ());
    SmartDashboard.putNumber("Error of ID " + limeSub.getId(), limeSub.getErrorValue());
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
