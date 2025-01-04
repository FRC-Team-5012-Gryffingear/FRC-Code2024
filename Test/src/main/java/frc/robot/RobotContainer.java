// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ModConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoSimple;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorComm;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HookCommand;
import frc.robot.commands.IntakeComm;
import frc.robot.commands.LedCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.limeCom;
import frc.robot.otherInfo.controllerConstant;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveMod;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.limey;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
  private final HookSubsystem hookSubsys = new HookSubsystem();
 private final LedSubsystem Ledsubsys = new LedSubsystem();

 private final limey lime = new limey();
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
  //  Ledsubsys.setDefaultCommand(new LedCommand(Ledsubsys));

    lime.setDefaultCommand(new limeCom(lime));
    
    swerveSubsys.setDefaultCommand(new SwerveCommand(swerveSubsys,
     driverController,
     () -> driverController.a().getAsBoolean(),
     () -> driverController.b().getAsBoolean()));

    intakeSub.setDefaultCommand(new IntakeComm(intakeSub,
    () -> operatorController.getAButton(),
    () -> operatorController.getBButton()));

    
     elevSub.setDefaultCommand(new ElevatorComm(elevSub,
     () -> operatorController.getRightTriggerAxis(), 
     () -> operatorController.getLeftTriggerAxis(),
     () -> operatorController.getLeftBumper(),
     () -> operatorController.getRightBumper()));

     //this is for the bag motor too hook on to the elevator
     hookSubsys.setDefaultCommand(new HookCommand(hookSubsys,
      () -> operatorController.getYButton(),
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
    //CREATING A TRAJECTORY RAHHHHH
    // 10 m/s^2 represents accel at reg grav basically
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.Max_velo, 
        10)
              .setKinematics(Constants.kinematics);
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
            List.of(
                      new Translation2d(1,0),
                      new Translation2d(1,-1)
            ),
            new Pose2d(2,-1,Rotation2d.fromDegrees(180)),
            trajectoryConfig
    );

    PIDController xController = new PIDController(ModConstants.KP, 0, 0);
    PIDController yController = new PIDController(ModConstants.KP, 0, 0);
    ProfiledPIDController thetacontroller  = new ProfiledPIDController(ModConstants.KP, 0, 0, ModConstants.thetaConstraints);

    thetacontroller.enableContinuousInput(-Math.PI, Math.PI);

    //Constructs a command to follow traj
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      swerveSubsys::getPose, // Use instance method
      Constants.kinematics,
      xController,
      yController,
      thetacontroller,
      swerveSubsys::setModStates, // Use instance method
      swerveSubsys);

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsys.resetPose(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsys.stopMods())
    );
  }
}
