// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.VisionComm;
import frc.robot.commands.elevCom;
import frc.robot.commands.intakeCom;
import frc.robot.otherInfo.controllerConstant;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.elevSS;
import frc.robot.subsystems.intakeSS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final elevSS subsys = new elevSS();
  private final intakeSS inss = new intakeSS();
  private final XboxController xbox = new XboxController(0);
  private final VisionSub subsyssss = new VisionSub();
  private final VisionComm commmmm = new VisionComm(subsyssss);


  public RobotContainer() {
    //subsys.setDefaultCommand(new elevCom(subsys, null, null));
    subsyssss.setDefaultCommand(new VisionComm(subsyssss));


    configureBindings();
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
    return commmmm;
  }
}
