// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.otherInfo.controllerConstant;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveMod;

import java.net.Authenticator.RequestorType;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private double Maxspeed = 6;
  private double MaxAngularRate = 1.5 * Math.PI;

  private final CommandXboxController driveController = new CommandXboxController(0);
  public final static CommandSwerveDrivetrain drivetrain = SwerveMod.train;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(Maxspeed * 0.15).withRotationalDeadband(MaxAngularRate*0.1)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
  }


  private void configureBindings() {

    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> drive.withVelocityX(-driveController.getLeftY() * Maxspeed)
      .withVelocityY(-driveController.getLeftX() * Maxspeed)
      .withRotationalRate(-driveController.getRightX()* MaxAngularRate)
      ).ignoringDisable(true));

      driveController.b().whileTrue(drivetrain
      .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));

      driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
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
