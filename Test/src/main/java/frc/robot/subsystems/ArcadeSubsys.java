// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArcadeSubsys extends SubsystemBase {
  /** Creates a new ArcadeSubsys. */
  TalonSRX talonFL = new TalonSRX(Constants.FL);
  TalonSRX talonFR = new TalonSRX(Constants.FR);
  TalonSRX talonBL = new TalonSRX(Constants.BR);
  TalonSRX talonBR = new TalonSRX(Constants.BL);

  TalonFX motor = new TalonFX(Constants.ran);

  public ArcadeSubsys() {
    talonFL.configFactoryDefault();
    talonFR.configFactoryDefault();
    talonBL.configFactoryDefault();
    talonBR.configFactoryDefault();

    motor.getConfigurator().apply(new TalonFXConfiguration());
    motor.setNeutralMode(NeutralModeValue.Brake);
    

    talonFL.setNeutralMode(NeutralMode.Coast);
    talonFR.setNeutralMode(NeutralMode.Coast);
    talonBL.setNeutralMode(NeutralMode.Coast);
    talonBR.setNeutralMode(NeutralMode.Coast);

    talonBL.follow(talonFL);
    talonBR.follow(talonFR);

    talonFR.setInverted(InvertType.InvertMotorOutput);
    talonBR.setInverted(InvertType.FollowMaster);

  }

  public void move_turn(double power, double turn){
    talonFL.set(ControlMode.PercentOutput, power + turn);
    talonFR.set(ControlMode.PercentOutput, power - turn);
  }
  public void button(boolean a){
    if(a){
      motor.set(1);
    }
    else{
      motor.set(0);
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
