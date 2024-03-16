// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class Autos extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final VisionSub m_subsystem;
  private Timer tier = new Timer();
  
  public Autos(VisionSub subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    System.out.println("INITIALIZING  AAAAA");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.startThread();
    //System.out.println("CHECK 111111111111111");
    /*
     * Error  = GOAl - Current
     * kp = something constant
     * 
     * power = kp * error
     * 
     * Actual stuff Distance z:
     * 
     * error = 85 - current firatees
     * 
     * kp = 0.2 needs to be changed
     * 
     * power1 = kp * error
     * 
     * --Actual for distance x:
     * 
     * error = 15 - current firatees
     * 
     * kp = 0.2 needs to be changed
     * 
     * power2 = kp * error
     * 
     * Actual roll:
     * 
     * error = 0.1 - abs current firatmeters
     * 
     * kp = 0.2 needs to be changed
     * 
     * power3 = kp * error
     * 
     */

    double Error5Z = 85 - m_subsystem.getZID(5);
    double Error6Z = 85 - m_subsystem.getZID(6);

    double kp = .2;

    double Zpower_ID_5 = kp * Error5Z;
    double Zpower_ID_6 = kp * Error6Z;

    double Error5X = 15 - m_subsystem.getXID(5);
    double Error6X = 15 - m_subsystem.getXID(6);

    double Xpower_ID_5 = kp * Error5X;
    double Xpower_ID_6 = kp * Error6X;

    double Error5Roll = .1 - m_subsystem.getIDroll(5);
    double Error6Roll = .1 - m_subsystem.getIDroll(6);

    double Rollpower_ID_5 = kp * Error5Roll;
    double Rollpower_ID_6 = kp * Error6Roll;

    if(Math.abs(Xpower_ID_5) >= 0){
      if(m_subsystem.getZID(5) < 90 && m_subsystem.getZID(5) > 83){
        Zpower_ID_5 = 0;
        
        System.out.println("Z IS 000000");
      }
      if (m_subsystem.getXID(5) > -9.25 && m_subsystem.getXID(5) < 15.25){
        Xpower_ID_5 = 0;
        System.out.println("THE X IS NOW 0");
      }
      if (m_subsystem.getIDroll(5) > -0.1 && m_subsystem.getIDroll(5) < 0.1){
        Rollpower_ID_5 = 0;
        System.out.println("THE ROLL IS PERFECTOR");
      }
  
      if(Zpower_ID_5 == 0 && Xpower_ID_5 == 0 && Rollpower_ID_5 == 0){
      System.out.println("ALLL VALUES ARE 0-------");

      //Outake that Note for X amount (To be determined)
      /*
       * if(time.get() > X seconds){
       * rotate 270 aka forward
       * Moveforward for X amount of time
       * }
       * 
       */
    }

        System.out.println("THIS IS THE Z NUMBER : " + Zpower_ID_5);
      System.out.println("THIS IS THE X NUMBER : " + Xpower_ID_5);
      System.out.println("THIS IS THE ROLL NUMBER : " + Rollpower_ID_5);
    } 

    //Timer thistimer = new Timer();
    //thistimer.start();
   


   // if (Math.abs(Xpower_ID_5) == 0 && Math.abs(Zpower_ID_5) == 0 && Math.abs(Error5Roll) == 0){
    // shooter = -1
    // shooter. stops
    //}
    //this is psuedocode for ID 5
    /* if (thistimer > 9.0){
      turn left
      move forward
      stop
    }
    // tHIS IS PSUEDOCODE FOR id 6
    /* if (thistimer > 9.0){
      turn right
      move forward
      stop
    }
     * 
     * 
     * 
     * 
     * 
     * 
     * 
     */


    // if(m_subsystem.getXID(5) > 15 && m_subsystem.getXID(5) < -9 || m_subsystem.getZID(5) > 85 || m_subsystem.getIDroll(5) < -0.01){
    //   System.out.println("ITS ACTUALLY WORKING ");
    // }


    // m_subsystem.startThread();
    // System.out.println(m_subsystem.getX_ID_5());
    // if(m_subsystem.getXID(5) < 15 & m_subsystem.getXID(5) > -9){
    //   System.out.println("WITHIN THRSHHHHHHHHHHHHHHHOOLLLLLLLLLLLLLLLDDDDDD");
    // }
    // else{
    //   System.out.println("ABOOOOVEEE THE THRESSHODSLDLSLSLSLLS");
    // }

    // System.out.println("Value of Z: " + m_subsystem.getZID(5));
    // if(m_subsystem.getZID(5) > 85){
    //   System.out.println("TOO FARRRRRRRRRRRRRRRRRRRRRRRRRRRR");
    // }
    // else {
    //   System.out.println("TOOOOOOOOOOOOOOOOOOOO CLOSEEEEEEEEEEEEEEEEEEEEE");
    // }

    // System.out.println(m_subsystem.getIDroll(5));
    // if(m_subsystem.getIDroll(5) < 0.1 & m_subsystem.getIDroll(5) > -0.01){
    //   System.out.println("WITHINNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN");
    // }
    // else{
    //   System.out.println("NOTTTTTTT THE THRESSHODSLDLSLSLSLLS");
    // }

    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopThread();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}