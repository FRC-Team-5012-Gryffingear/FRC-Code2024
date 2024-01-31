// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  TalonSRX leftMotor = new TalonSRX(0);
  TalonSRX rightMotor = new TalonSRX(0);


  public Elevator() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setInverted(InvertType.InvertMotorOutput);
    leftMotor.follow(rightMotor);
  }

  
  public void power(boolean up, boolean down){
    rightMotor.set(ControlMode.PercentOutput, 0);
    if(up){
        rightMotor.set(ControlMode.PercentOutput, 1);
    }
    else if(down){
        rightMotor.set(ControlMode.PercentOutput, -1);
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
