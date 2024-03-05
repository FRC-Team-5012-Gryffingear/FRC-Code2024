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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsys extends SubsystemBase {
  /** Creates a new ElevatorSubsys. */
  TalonSRX elevMotor1 = new TalonSRX(Constants.Elev1);
  TalonSRX elevMotor2 = new TalonSRX(Constants.Elev2);

  DigitalInput Limit1Top = new DigitalInput(2);
 
  Timer time = new Timer();

  public ElevatorSubsys() {
    // elevMotor1.configFactoryDefault();
    // elevMotor2.configFactoryDefault();

    elevMotor1.setNeutralMode(NeutralMode.Brake);
    elevMotor2.setNeutralMode(NeutralMode.Brake);

    elevMotor2.setInverted(InvertType.InvertMotorOutput);
    elevMotor2.follow(elevMotor1);
    //If motor right side then do invert if on left then do not invert
    //Ignore if gears make them inverted already and


    
  }



  public void elevating(double power, boolean push){
    if(push){
      elevMotor1.set(ControlMode.PercentOutput, power*2);
      System.out.println("Inside movement basically");
    }
//Checks top limit and if power is positive
//if it is to turn it completely off 
    else if(!Limit1Top.get() && power > 0){
      elevMotor1.set(ControlMode.PercentOutput, 0);
    }
// if the button is not pressed, top and lower limits not being pressed
// to move normally since none of the requirements are being met
    else{
      elevMotor1.set(ControlMode.PercentOutput, power*0.8); 
    }

  }


  public void elevating2(double power,boolean push){
    if(push){
System.out.println("Nothing for now");
    }
    else{
      time.stop();
      time.reset();
      elevMotor1.set(ControlMode.PercentOutput , power*0.8);
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
