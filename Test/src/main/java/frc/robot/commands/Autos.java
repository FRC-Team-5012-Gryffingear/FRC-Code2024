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
    /* Error = GOAL - Current
     * kp = something constant
     * 
     * power = kp * error
     * 
     * --Actual stuff Distance z:
     * 
     * error = 85 - current firatees
     * 
     * kp = 0.2 needs to be changed
     * 
     * power1 = kp * error
     * 
     * --Actual for Distance x;
     * 
     * error = 15 - current firatees
     * 
     * kp = 0.2 needs to be changed
     * 
     * power2 = kp * error
     * 
     * --Actual roll:
     * 
     * error = 0.1 - abs current firatmeters
     * 
     * kp = 0.2 needs to be changed
     * 
     * power3 = kp * error
     * 
     */
    System.out.println("FEVERD REAAMM");
    vision.startVision();
    System.out.println(vision.get_ID_Xpose(5));
    System.out.println("PASSSING");
    //Changes: Created a VisionComm where we would initiate the thread plus
    //initiate the function. Not completely sure if this works, but hopefully it does.
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.stopVision();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
