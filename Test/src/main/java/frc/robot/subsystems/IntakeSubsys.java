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
import edu.wpi.first.wpilibj.Timer;


public class IntakeSubsys extends SubsystemBase {
    TalonSRX intake1 = new TalonSRX(Constants.Intake1);
    private Timer timer1 = new Timer();
    //have left side motor (Right side when in front) be the intake1
  public IntakeSubsys() {
    intake1.configFactoryDefault();

    intake1.setNeutralMode(NeutralMode.Brake);
    intake1.setInverted(InvertType.InvertMotorOutput);
  }

//Check if timer actually works on intake
  public void intaking(boolean a, boolean b){
    if(a){
      // Intaking
      timer1.start();
      if(timer1.get() < .3) {
          intake1.set(ControlMode.PercentOutput, 1);
       }
       else{
        intake1.set(ControlMode.PercentOutput, 0);
       }
    }
    else if(b){
      //Outtaking
        intake1.set(ControlMode.PercentOutput, -1);
    }
    else{
        intake1.set(ControlMode.PercentOutput, 0);
        timer1.stop();
        timer1.reset();
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
