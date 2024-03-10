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
import edu.wpi.first.wpilibj.Timer;


public class IntakeSubsys extends SubsystemBase {
    TalonSRX intake1 = new TalonSRX(Constants.Intake1);
    DigitalInput intakeLimit = new DigitalInput(1);
   // private Timer timer1 = new Timer();
    //have left side motor (Right side when in front) be the intake1
  public IntakeSubsys() {
  //  intake1.configFactoryDefault();

    intake1.setNeutralMode(NeutralMode.Brake);
   // intake1.setInverted(InvertType.InvertMotorOutput);
  }




  public void intaking(boolean a, boolean b){
    //Checks if the A button is being pressed 
    if(a){
      //once the A button is pressed also check if the intake limit is pressed to see if we hit our goal
      if(intakeLimit.get()==true){
        intake1.set(ControlMode.PercentOutput, 1);
      }
      //if not then just continue intaking until true
      else{
        intake1.set(ControlMode.PercentOutput, 0);
      }
      
    }
    //Check if B button is pressed
    else if(b){
      //if it is then outtake that note out
      intake1.set(ControlMode.PercentOutput, -1);
    }
    //Nothing pressed? Don't move!
    else{
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
