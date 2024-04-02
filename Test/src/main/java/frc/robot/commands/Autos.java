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
  private boolean too_close = false;
      private boolean checker;

 
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

    checker = false;

    elevTime.reset();
    elevTime.stop();

    elev.Checkoff();
    //System.out.println("Runnnnnnninnnnng");
    swerve.resetHeading();
    vision.startThread();
  }

     // double thres = X/Z
  public boolean AutoMovement(double Error5X, double finalPushX, double Error5Z){

//Change these values to ID 6 since it is on the left side
        //next comment may or may not be correct:
        double relationship = (2.48456 * (Error5Z * 12)) + 25.9588;
        System.out.println("THIS IS RELATIONSHIP ------: " + relationship);
        if((relationship/2) < 40){
          relationship = 40;
        }
        if(-relationship/2 < vision.getXID(5) && vision.getXID(5) < relationship/2){
          if(-0.1 < vision.getIDroll(5) && vision.getIDroll(5) < 0.1){
            System.out.println("CHECKER ONNNNNNNNNNNNNNNNNNNNNNNnn");
            checker = true;
            return checker;
          }
          else{
            Error5X = 0;
            if(vision.getIDroll(5) < 0){
              Timer.stop();
              /*
              * Divide finalPushX by 1.75 if too fast
              */
              swerve.drive3(0.02, 0, 0, false);  
            }
            else if(0 < vision.getIDroll(5)){
              Timer.stop();
              /*
              * Divide finalPushX by 1.75 if too fast
              */
              swerve.drive3(-0.02, 0, 0, false);
            }
          }
        }
        else{
          Timer.stop();
          /*
           * Replace with finalPushX
           * Divide finalPushX by 1.75 if too fast
           */
          swerve.drive3(0,0,finalPushX/10,false);

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

    //this section is for X
    //Change 15 to a number that takes in count the offset of the camera since it is on the side
    //subtract 15 to correct the offset and divide by amount of units that equals one foot for X axis
    double Error5X = vision.getXID(5)/61.7;
    double Error6X = vision.getXID(6)/61.7;

    //This section is for Roll (rotating)
    //Might cause an error since the value read is 0 - 360 (Maybe)
    //Check to see if changing the Mod encoder config to -pi to pi works
    double Error5Roll = vision.getIDroll(5);    
    double Error6Roll = vision.getIDroll(6) - .1;

    // Checks to see if last known robot to April Tag position is too close to April Tag
    if(Math.abs(Error5Z) < 100 && Math.abs(Error5X) < 25 && Math.abs(Error5Roll) < 0.1){
      too_close = true;
    }

    else
    {
      too_close = false;
    }

    //Add Threshold checkers  for ID 5 & 6 here:

     double alpha = 0.001; //changed from .015 // Alpha goes up causes finalPush values to get smaller and vice versa

    
    // Change this to abs.() < 0.02 later if when camera is close to April Tag the robot moves weird (like too fast or too small)
     if(Math.abs(Error5Z) == 0 && Math.abs(Error5X) == 0 && Math.abs(Error5Roll) == 0){
      // Checks to see if last recorded position is too close to the April Tag
      if (too_close) {
        t = t/2;
      }
      
      else {
        t = 1;
      } 
    }

    double finalPushZ = Math.pow(Math.E, -alpha*t*Error5Z);
    double finalPushX = Math.pow(Math.E, -alpha*t*Error5X);
    double finalPushRoll = Math.pow(Math.E, -alpha*t*Error5Roll);

    /*
     * Bring back this code if the speeds for Z, X, and Roll get super fast
     */
    // if(finalPushX > .1){ //changed, used to be finalPushX > 1
    //   finalPushX = .1; 
    // }
    // else if(finalPushX < -.1){
    //   finalPushX = -.1;
    // }


    // if(finalPushZ > .1){
    //   finalPushZ = .1;
    // }
    // else if(finalPushZ < -.1){
    //   finalPushZ = -.1;
    // }

  
    // if(finalPushRoll > .1){ //changed, used to be finalPushRoll > 1
    //   finalPushRoll = .1; 
    // }
    // else if(finalPushRoll < -.1){
    //   finalPushRoll = -.1;
    // }

    boolean rollCheck = false;
    boolean xCheck = false;


    
/*
 * v^2 = v_0^2 + 2a(x - x_0)
 * v0 is initial velocity, v is final velocity, a is acceleration, X is final position, and x0 is initial position
 */
    /*
     * Change finalPushZ to Error5Z/3.2808 if too fast
     */
    double v = Math.sqrt(2*0.02*Error5Z/3.2808);
    

    System.out.println("---------THIS IS V VALUE: " + v);
    time = ((Error5Z/3.2808)/v);

    System.out.println("-------------This is TIme: " + time/3);




//Vision with roll and elevator

  elevTime.start();
  // System.out.println(elevTime.get() + "Timer testing -a-a-a-a-a-a-a-a-a-");
  if(elevTime.get() < 1){
  // elev.elevating(-num,  false, false);
  elev.elevatingAuto(-0.8, false);
  }
  if(elevTime.get() > 1){
    elev.elevatingAuto(0.8, false);
  if(!elev.elevLimit()){
    // elev.elevating(0.1, true, false);
    elev.elevatingAuto(0.1, false);
  }
  // System.out.println(elevTime.get() + "Timer testing -a-a-a-a-a-a-a-a-a-");
}


    if(Math.abs(SmartDashboard.getNumber("ID 5 X Value", 0)) > 0){
          



    // AutoMovement(Error5X, finalPushX,Error5Z);

    while(checker){

      // System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
      // if(vision.getZID(5) > 85){
      //   swerve.drive3(0, -0.05, 0, false);
      // }else{
      //   swerve.drive3(0, 0, 0, false);
      // }
    }
    //         // Timer.start();
    //         // System.out.println("--------------This is Timer: " + Timer.get()*1);
    //         // if(Timer.get()*1 < (time)){
    //         //   /*
    //         //    * divide v by 3.25 if too fast
    //         //    */
    //         //   swerve.drive3(0, -v/3.25, 0, false);
    //         //   System.out.println("WITHIN THE THRESH");
    //         // }
    //         // else{
    //         //   swerve.drive3(0, 0, 0, false);
    //         //   //add outake code here
    //         // }
    // }

      




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
    else {
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