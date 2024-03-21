// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSub;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveCommand;
/** An example command that uses an example subsystem. */
public class Autos extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerve;
  private final VisionSub vision;
  private final ElevatorSubsys elev;
  //t is different from Time :)
  private double t = 1;
  private double time;
  private Timer Timer = new Timer();
  private Timer elevTime = new Timer();

 
  public Autos(SwerveSubsystem subsystem, VisionSub vision, ElevatorSubsys elev) {
    swerve = subsystem;
    this.vision = vision;
    this.elev = elev;
    addRequirements(subsystem,vision);
  }



  @Override
  public void initialize() {
    Timer.reset();
    Timer.stop();

    elevTime.reset();
    elevTime.stop();

    elev.Checkoff();
    //System.out.println("Runnnnnnninnnnng");
    swerve.resetHeading();
    vision.startThread();
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
  public boolean AutoMovement(double Error5X, double finalPushX){
    boolean checker = false;
//Change these values to ID 6 since it is on the left side
        if(-15 < vision.getXID(5) && vision.getXID(5) < 75){
          if(-0.1 < vision.getIDroll(5) && vision.getIDroll(5) < 0.1){
            System.out.println("CHECKER ONNNNNNNNNNNNNNNNNNNNNNNnn");
            checker = true;
            return checker;
          }
          else{
            Error5X = 0;
            if(vision.getIDroll(5) < 0){
              Timer.stop();
              swerve.drive3(.02, 0, 0, false);  
            }
            if(0 < vision.getIDroll(5)){
              Timer.stop();
              swerve.drive3(-.02, 0, 0, false);
            }
          }
        }
        else{
          Timer.stop();
          swerve.drive3(0,0,finalPushX/1.75,false);
        }
        return checker;
  }

  

  @Override
  public void execute() {
    //vision.startVision();

    //ALL OF THE GOALS AND THRESHOLDS NEEED TO BE CHANGED OR WILL NOT FUNCTION DUE TO THE NEW METHOD
    //BEING IMPLEMENTED
    //Z quick measure was around 40 - 50 firameters
    //Roll did not seem to change but needs to be tested
    //X was not checked but seems to be centered at (0,0) Maybe

  
    //FOR AMPS DO TAGS 5 AND 3
    //THIS SECTION IS FOR Z
    //THESE ARE FOR PID (MIGHT NEED TO CHANGE LATER)
    //1ft = 85 units
    double Error5Z = vision.getZID(5)/41;
    double Error6Z = vision.getZID(6)/41;

    //this Kp value will be permanent for all values
    double kp = 0.4;

    // double Zpower_ID_5 = kp * Error5Z; 
    // double Zpower_ID_6 = kp * Error6Z;

    //this section is for X
    //Change 15 to a number that takes in count the offset of the camera since it is on the side
    //subtract 15 to correct the offset and divide by amount of units that equals one foot for X axis
    double Error5X = vision.getXID(5)/61.7;
    double Error6X = vision.getXID(6)/61.7;

    // double Xpower_ID_5 = kp * Error5X;
    // double Xpower_ID_6 = kp * Error6X;

    //This section is for Roll (rotating)
    //Might cause an error since the value read is 0 - 360 (Maybe)
    //Check to see if changing the Mod encoder config to -pi to pi works
    double Error5Roll = vision.getIDroll(5);    
    double Error6Roll = vision.getIDroll(6) - .1;

    // double Rollpower_ID_5 = kp * Error5Roll;
    // double Rollpower_ID_6 = kp * Error6Roll;

    //Add Threshold checkers  for ID 5 & 6 here:

     double alpha = 0.001; //changed from .015

    if(Math.abs(Error5Z) < 0.02 && Math.abs(Error5X) < 0.02 && Error5Roll < 0.02){
      t = 1; 
    }

    double finalPushZ = 1 / (1 - Math.pow(Math.E, -alpha*t*Error5Z));
    double finalPushX = 1 / (1 - Math.pow(Math.E, -alpha*t*Error5X));
    double finalPushRoll = 1 / (1 - Math.pow(Math.E, -alpha*t*Error5Roll));

    if(finalPushX > .1){ //changed, used to be finalPushX > 1
      finalPushX = .1; 
    }
    else if(finalPushX < -.1){
      finalPushX = -.1;
    }


    if(finalPushZ > .1){
      finalPushZ = .1;
    }
    else if(finalPushZ < -.1){
      finalPushZ = -.1;
    }

  
    if(finalPushRoll > .1){ //changed, used to be finalPushRoll > 1
      finalPushRoll = .1; 
    }
    else if(finalPushRoll < -.1){
      finalPushRoll = -.1;
    }

    boolean rollCheck = false;
    boolean xCheck = false;


    
/*
 * v^2 = v_0^2 + 2a(x - x_0)
 * v0 is initial velocity, v is final velocity, a is acceleration, X is final position, and x0 is initial position
 */
    double v = Math.sqrt(2*0.02*Error5Z/3.2808);
    

    System.out.println("---------THIS IS V VALUE: " + v);
    time = ((Error5Z/3.2808)/v);

    System.out.println("-------------This is TIme: " + time/3);


    if(Math.abs(SmartDashboard.getNumber("ID 5 X Value", 0)) > 0){

      // Timer.start();
      // System.out.println("--------------This is Timer: " + Timer.get()*1.5);
      // if(Timer.get()*1.25 < (time)){
      //   swerve.drive3(v/4, 0, 0, true);
      //   System.out.println("WITHIN THE THRESH");
      // }
      // else{
      //   swerve.drive3(0, 0, 0, true);
      // }


          

//Vision with roll and elevator
  elevTime.start();

  System.out.println(elevTime.get() + "Timer testing -a-a-a-a-a-a-a-a-a-");
  elev.elevating(-0.8, false, false, true);
  if(elevTime.get() > 1){
    elev.elevating(0.8, false, true, false);
  if(!elev.elevLimit()){
    elev.elevating(0.1, false, true, false);
  }
  System.out.println(elevTime.get() + "Timer testing -a-a-a-a-a-a-a-a-a-");
}


    AutoMovement(Error5X, finalPushX);
    if(AutoMovement(Error5X, finalPushX)){
      System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
            Timer.start();
            System.out.println("--------------This is Timer: " + Timer.get()*1.25);
            if(Timer.get()*1.25 < (time)){
              swerve.drive3(0, -v/3.25, 0, false);
              System.out.println("WITHIN THE THRESH");
            }
            else{
              swerve.drive3(0, 0, 0, false);
              //add outake code here
            }
    }

      




      // if(xCheck && rollCheck){
      //   swerve.drive3(0, 0, 0, true);
      // }
      // swerve.drive3(0, -Error5X, Error5Roll, true);
      // if( 39 < vision.getZID(5)  && vision.getZID(5) < 43){
      //   Error5Z = 0;
      //   swerve.drive3(Error5Z, Error5X, Error5Roll, true);
      //   System.out.println("Z IS 000000");
      // }




      // if(-15 < vision.getXID(5) && vision.getXID(5) < 75){
      //   Error5X = 0;
      //   swerve.drive3(0, 0, 0, true);
      //   System.out.println("THE X IS NOW 0");
      //   xCheck = true;
      // }
      // // else if(vision.getXID(5) < 25){
      // //   swerve.drive3(0, -Math.abs(finalPushX/2), 0, true);
      // // }
      // else{
      //   swerve.drive3(0, finalPushX/2, 0, true);
      //   xCheck =false;
      // }


      
// ROLL NEEDS TO BE FIXED AGAIN


    //   if( 0.45 < vision.getIDroll(5) && vision.getIDroll(5) < 0.75){
    //     rollCheck = true;
    //     Error5Roll = 0;
    //     swerve.drive3(0, 0, 0, true);
    //      System.out.println("THE ROLL IS PERFECTOR");
    //  }
    //  else if(vision.getIDroll(5) < 0.45){
    //  swerve.drive3(0, 0, -Math.abs(finalPushRoll/1.75) , true); 
    //  rollCheck = false;
    //  }

    //  else{
    //   rollCheck = false;
    //   System.out.println("THIS IS FINAL PUSH: " + finalPushRoll);
    //   swerve.drive3(0, 0, finalPushRoll/1.75 , true);
    //  }

     t = t + 1;

     
   
    }
    else{
      swerve.drive3(0, 0, 0, true);
      t = 1;
    }

    // if(Error5X == 0 && Error5Roll ==0){
    //   swerve.drive3(0, 0, 0, true);
    // }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //  vision.stopVision();
  vision.stopThread();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
