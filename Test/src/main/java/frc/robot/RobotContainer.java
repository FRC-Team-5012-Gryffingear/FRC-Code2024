// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  SwerveSubsystem swerveSubsys = new SwerveSubsystem();


  CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DriverControllerPort);

  SwerveCommand swerveCom = new SwerveCommand(swerveSubsys, driverController);


  public RobotContainer() {
    swerveSubsys.setDefaultCommand(swerveCom);

    configureBindings();
  }


  private void configureBindings() {
    driverController.rightBumper().onTrue(new InstantCommand(swerveSubsys::resetHeading, swerveSubsys));

    driverController.y().onTrue(new InstantCommand(swerveSubsys::resetPose, swerveSubsys));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(null);
  }
}
