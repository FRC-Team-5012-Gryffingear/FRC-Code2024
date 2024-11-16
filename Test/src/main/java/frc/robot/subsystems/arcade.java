// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arcade extends SubsystemBase {
  /** Creates a new arcade. */
  TalonSRX mot1 = new TalonSRX(Constants.motor1);
  TalonSRX mot2 = new TalonSRX(Constants.motor2);
  TalonSRX mot3 = new TalonSRX(Constants.motor3);
  TalonSRX mot4 = new TalonSRX(Constants.motor4);

  public arcade() {
    mot1.configFactoryDefault();
    mot2.configFactoryDefault();
    mot3.configFactoryDefault();
    mot4.configFactoryDefault();

    mot1.setNeutralMode(NeutralMode.Brake);
    mot2.setNeutralMode(NeutralMode.Brake);
    mot3.setNeutralMode(NeutralMode.Brake);
    mot4.setNeutralMode(NeutralMode.Brake);

    mot1.setInverted(InvertType.InvertMotorOutput);
    mot3.setInverted(InvertType.InvertMotorOutput);
    mot3.follow(mot1);

    mot4.follow(mot2);


  }

  public void move(double x, double turn){
    mot1.set(ControlMode.PercentOutput, x - turn);
    mot2.set(ControlMode.PercentOutput, x + turn);
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
