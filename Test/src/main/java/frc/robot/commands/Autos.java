// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSub;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Autos extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerve;
  private final VisionSub vision;

 
  public Autos(SwerveSubsystem subsystem, VisionSub vision) {
    swerve = subsystem;
    this.vision = vision;


    addRequirements(subsystem,vision);
  }

  @Override
  public void initialize() {
    System.out.println("Runnnnnnninnnnng");
  }

  @Override
  public void execute() {
    System.out.println("FEVERD REAAMM");
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
