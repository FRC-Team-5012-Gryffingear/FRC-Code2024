// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class SwerveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 private final SwerveSubsystem swervesubsys;
 private final CommandXboxController controller2;

 
  
  public SwerveCommand(SwerveSubsystem subsystem, CommandXboxController controller) {
    swervesubsys = subsystem;
    controller2 = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -MathUtil.applyDeadband(controller2.getLeftY() * 2, OperatorConstants.DriverDeadband) * 0.4;
    double ySpeed = MathUtil.applyDeadband(controller2.getLeftX() * 2, OperatorConstants.DriverDeadband) * 0.4;
    double rotateSpeed = MathUtil.applyDeadband(controller2.getRightX() * 2, OperatorConstants.DriverDeadband) * 0.4;

    swervesubsys.drive(xSpeed, ySpeed, rotateSpeed,true);
    
    // if(controller2.rightBumper().getAsBoolean()){
    //     swervesubsys.drive(xSpeed, ySpeed, rotateSpeed,true);
    // } else{
    //     swervesubsys.drive(xSpeed, ySpeed, rotateSpeed);
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swervesubsys.stopMods();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
