// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoSimple;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorComm;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeComm;
import frc.robot.commands.SwerveCommand;
import frc.robot.otherInfo.controllerConstant;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSub;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
  

  private final SwerveSubsystem swerveSubsys = new SwerveSubsystem();
  private final IntakeSubsys intakeSub = new IntakeSubsys();
  private final ElevatorSubsys elevSub = new ElevatorSubsys();
  private final VisionSub visionSub = new VisionSub();

  private final Autos auto = new Autos(swerveSubsys, visionSub);
  private final AutoSimple AutoS = new AutoSimple(swerveSubsys,intakeSub,elevSub);




  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DriverControllerPort);
  // XboxController operatorController =
  //     new XboxController(OperatorConstants.OperatorControllerPort);
  
 private final XboxController operatorController = new XboxController(OperatorConstants.OperatorControllerPort);

  //SwerveCommand swerveCom = new SwerveCommand(swerveSubsys, driverController);


  public RobotContainer() {
    configureBindings();
    
    //Check if A and B register since we switched to Xboxcontroller from CommandXboxcontroller
    
    swerveSubsys.setDefaultCommand(new SwerveCommand(swerveSubsys,
     driverController,
     () -> driverController.a().getAsBoolean()));

    intakeSub.setDefaultCommand(new IntakeComm(intakeSub,
    () -> operatorController.getAButton(),
    () -> operatorController.getBButton()));

    
     elevSub.setDefaultCommand(new ElevatorComm(elevSub,
     () -> operatorController.getRightTriggerAxis(), 
     () -> operatorController.getLeftTriggerAxis(),
     () -> operatorController.getXButton()));

  }



  private void configureBindings() { 

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(null);
    return null;
  }
}
