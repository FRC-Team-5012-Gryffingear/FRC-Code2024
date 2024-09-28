// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArcadeDriv extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonSRX talonFL = new TalonSRX(Constants.FL);
  TalonSRX talonBL = new TalonSRX(Constants.BL);
  TalonSRX talonFR = new TalonSRX(Constants.FR);
  TalonSRX talonBR = new TalonSRX(Constants.BR);

  
  public ArcadeDriv() {
    talonFL.configFactoryDefault();
    talonBL.configFactoryDefault();
    talonFR.configFactoryDefault();
    talonBR.configFactoryDefault();

    talonFL.setNeutralMode(NeutralMode.Brake);
    talonBL.setNeutralMode(NeutralMode.Brake);
    talonFR.setNeutralMode(NeutralMode.Brake);
    talonBR.setNeutralMode(NeutralMode.Brake);

    talonBL.follow(talonFL);
    talonBR.follow(talonFR);

    talonFR.setInverted(InvertType.InvertMotorOutput);
    talonBR.setInverted(InvertType.FollowMaster);
  }

  public void moveandturn(double power, double turn){
    talonFL.set(ControlMode.PercentOutput, power + turn);
    talonFR.set(ControlMode.PercentOutput, power - turn);
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
