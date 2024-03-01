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

  DigitalInput Limit1Top = new DigitalInput(0);
  DigitalInput Limit2Bottom = new DigitalInput(1);
  Timer time = new Timer();

  public ElevatorSubsys() {
    elevMotor1.configFactoryDefault();
    elevMotor2.configFactoryDefault();

    elevMotor1.setNeutralMode(NeutralMode.Brake);
    elevMotor2.setNeutralMode(NeutralMode.Brake);

    elevMotor2.follow(elevMotor1);
    //If motor right side then do invert if on left then do not invert
    //Ignore if Gears invert the way it works
    elevMotor1.setInverted(InvertType.InvertMotorOutput);

    
  }

//Has "push" as a button and once pressed it moves up for 0.3 seconds
//"Power" is the regular supplier for the elevator that moves when Bottom or Top is not currently being pressed
//
  public void elevating(double power, boolean push){
    if(push){
      time.start();
      if(time.get() < 0.3){
        if(Limit1Top.get()){
          elevMotor1.set(ControlMode.PercentOutput, 0);
        }
        else{
          elevMotor1.set(ControlMode.PercentOutput, .25);
        }
      }
      
      else if(time.get() > 0.3){
        elevMotor1.set(ControlMode.PercentOutput, 0);
      }
    }
//Checks top limit and if power is being supplied 
    else if(Limit1Top.get() && power > 0){
      elevMotor1.set(ControlMode.PercentOutput, 0);
    }

    else if(Limit2Bottom.get() && power < 0){
      elevMotor2.set(ControlMode.PercentOutput, 0);
    }

    else{
      elevMotor1.set(ControlMode.PercentOutput, power/2); 
    }
    time.stop();
    time.reset();
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
