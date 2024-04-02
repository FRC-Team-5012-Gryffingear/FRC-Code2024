// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorSubsys extends SubsystemBase {
  /** Creates a new ElevatorSubsys. */
 // boolean check = true;
  boolean presser;
  boolean check;
  TalonSRX elevMotor1 = new TalonSRX(Constants.Elev1);
  TalonSRX elevMotor2 = new TalonSRX(Constants.Elev2);

  private DigitalOutput LED2 = new DigitalOutput(2);
  private Timer time2 = new Timer();

  private DigitalOutput LED4 = new DigitalOutput(4);

  // private LedSubsystem ledSubsystem = new LedSubsystem();

  DigitalInput Limit1Top = new DigitalInput(6);
 // DigitalInput Limit2Bottom = new DigitalInput(3);
 
  Timer time = new Timer();

  public ElevatorSubsys() {
    // elevMotor1.configFactoryDefault();
    // elevMotor2.configFactoryDefault();
    presser = false;
    // presser2 = false;
    check = true;

    elevMotor1.setNeutralMode(NeutralMode.Brake);
    elevMotor2.setNeutralMode(NeutralMode.Brake);

    elevMotor2.setInverted(InvertType.InvertMotorOutput);
    elevMotor2.follow(elevMotor1);
    //If motor right side then do invert if on left then do not invert
    //Ignore if gears make them inverted already and

    //Green = 2


    
  }
  public void Checkoff(){
    check = true;
  }

  public boolean elevLimit(){
    if(Limit1Top.get() == false){
      check = false;
      return check;
    }
    else{
      if(!check){
        return false;
      }
      check = true;
      return true;
    }
  }

  private void LED_on4(){
    LED4.set(true);
  }
  private void LED_off4(){
    LED4.set(false);
  }


  private void LED_on(){
    LED2.set(true);
  }
  private void LED_off(){
    LED2.set(false);
  }
  private void LED_blink(){
    time.start();
    if(time.get() > 0 && time.get() < 0.0625){
      LED_on();
    }
    if(time.get() > 0.25){
      LED_off();
      if(time.get() > 0.625){
        time.reset();
      }
    }
  }


  public void elevating(double power, boolean stall, boolean stall_off){
//Checks top limit and if power is positive
//if it is to turn it completely off 
System.out.println("This is POWERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR: " + power);
    // System.out.println("-------Limit Switch2 ELEVATOR: " + Limit1Top.getChannel() + "    VALUE: " + Limit1Top.get());
    if(!Limit1Top.get() && power > 0){
      elevMotor1.set(ControlMode.PercentOutput, 0);
      LED_on();
      if(presser){
        elevMotor1.set(ControlMode.PercentOutput, 0.1);
      }
    }
    //  else if(!Limit2Bottom.get() && power < 0){
    //  elevMotor1.set(ControlMode.PercentOutput, 0);
    //  }
// if the button is not pressed, top and lower limits not being pressed
// to move normally since none of the requirements are being met
    else if(Math.abs(power) > 0){
      elevMotor1.set(ControlMode.PercentOutput, power*0.8); 
      LED_blink();
    }
    else{
      if(stall){
        presser = true;
        // presser2 = false;
        elevMotor1.set(ControlMode.PercentOutput, 0.1);
      }
      else if(stall_off){
        presser = false;
        // presser2 = true;
        elevMotor1.set(ControlMode.PercentOutput, 0);
      }

      if(presser){
        elevMotor1.set(ControlMode.PercentOutput, 0.1);
        LED_on4();
      }
      else{
         elevMotor1.set(ControlMode.PercentOutput, 0);
        LED_off4();        
      }

      LED_off();

    }
  }


  public void elevatingAuto(double power,boolean push){
      elevMotor1.set(ControlMode.PercentOutput , power*0.8);
      if(!Limit1Top.get() && power > 0){
        elevMotor1.set(ControlMode.PercentOutput, 0.1);
      }
      System.out.println(power + " Power -da-d-da--d-d-d-d--d-d");
  }
  // public boolean bottomLimitBOOL(){
  //   return Limit2Bottom.get();
  // }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
