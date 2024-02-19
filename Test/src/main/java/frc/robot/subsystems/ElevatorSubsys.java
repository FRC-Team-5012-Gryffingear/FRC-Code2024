// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsys extends SubsystemBase {
  /** Creates a new ElevatorSubsys. */

  TalonSRX elevMotor1 = new TalonSRX(Constants.Elev1);

  public ElevatorSubsys() {
    elevMotor1.configFactoryDefault();

    elevMotor1.setNeutralMode(NeutralMode.Brake);
    
    elevMotor1.setInverted(InvertType.InvertMotorOutput);
  }


  public void elevating(double power){
    elevMotor1.set(ControlMode.PercentOutput, power);
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
