// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class AutoSimple extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerve;
  private final IntakeSubsys intakeSubsys;
  private final ElevatorSubsys ElevSubsys;

  private Timer time = new Timer(); 
  public AutoSimple(SwerveSubsystem subsystem, IntakeSubsys intakeSubsystem, ElevatorSubsys elevSubsys) {
    swerve = subsystem;
    intakeSubsys = intakeSubsystem;
    ElevSubsys = elevSubsys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Makes its forward new forward and resets timer
    swerve.resetHeading();
    time.stop();
    time.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this math should autocorrect the yaw of the robot to match its forward taking account any broken modules(Should)
    double percentRot = swerve.getYaw()/10;
    //Ex: swerve.drive3(0,0,percent,true); 




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive3(0, 0, 0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
