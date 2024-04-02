// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;


public class IntakeSubsys extends SubsystemBase {
    TalonSRX intake1 = new TalonSRX(Constants.Intake1);
    DigitalInput intakeLimit = new DigitalInput(5);
   private DigitalOutput LED3 = new DigitalOutput(3);
   private Timer time = new Timer();
  
//blue = 3

   // private Timer timer1 = new Timer();
    //have left side motor (Right side when in front) be the intake1
  public IntakeSubsys() {
  //  intake1.configFactoryDefault();

    intake1.setNeutralMode(NeutralMode.Brake);
    intake1.setInverted(InvertType.InvertMotorOutput);
  }

  private void LED_on() {
    LED3.set(true);
    }
  private void LED_off(){
    LED3.set(false);
  }
  private void LED_blink(){
    time.start();
    if(time.get() > 0 && time.get() < 0.0625){
      LED3.set(true);
    }
    if(time.get() > 0.25){
      LED3.set(false);
      if(time.get() > 0.625){
        time.reset();
      }
  }
}



  public void intaking(boolean a, boolean b){
    //Checks if the A button is being pressed 
  //  System.out.println("------LIMIT SWITCH DEBUG INTAKE: " + intakeLimit.getChannel());
    if(a){
      //once the A button is pressed also check if the intake limit is pressed to see if we hit our goal
      if(intakeLimit.get()==true){
        intake1.set(ControlMode.PercentOutput, 1);
        LED_blink();
        
      }
      //if not then just continue intaking until true
      else{
        LED_on();
        intake1.set(ControlMode.PercentOutput, 0);
      }
      
    }
    //Check if B button is pressed
    else if(b){
      //if it is then outtake that note out
      intake1.set(ControlMode.PercentOutput, -1);
      LED_blink();
    }
    //Nothing pressed? Don't move!
    else{
      if(intakeLimit.get() == true){
        
        LED_off();
      }
        intake1.set(ControlMode.PercentOutput, 0);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
