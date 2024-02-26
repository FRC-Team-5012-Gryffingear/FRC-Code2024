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
  @Override
  public void execute() {
    vision.startVision();
  
    //FOR AMPS DO TAGS 5 AND 3
    //THIS SECTION IS FOR Z
    //THESE ARE FOR PID (MIGHT NEED TO CHANGE LATER)
    double Error5Z = 85 - vision.getZID(5);
    double Error3Z = 85 - vision.getZID(3);

    //this Kp value will be permanent for all values
    double kp = 0.2;

    double Zpower_ID_5 = kp * Error5Z; 
    double Zpower_ID_3 = kp * Error3Z;

    //this section is for X
    //Change 15 to a number that takes in count the offset of the camera since it is on the side
    double Error5X = 15 - vision.getXID(5);
    double Error3X = 15 - vision.getXID(3);

    double Xpower_ID_5 = kp * Error5X;
    double Xpower_ID_3 = kp * Error3X;

    //This section is for Roll (rotating)
    //Might cause an error since the value read is 0 - 360 (Maybe)
    //Check to see if changing the Mod encoder config to -pi to pi works
    double Error5Roll = 0.1 - vision.getIDroll(5);
    double Error3Roll = 0.1 - vision.getIDroll(3);

    double Rollpower_ID_5 = kp * Error5Roll;
    double Rollpower_ID_3 = kp * Error3Roll;
    //Application of these variables will probably look like
    


    //Changes: Created a VisionComm where we would initiate the thread plus
    //initiate the function.
    
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
