// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Split extends SubsystemBase {
  /** Creates a new Split. */
  DigitalInput topLimit = new DigitalInput(0);
  DigitalInput bottomLimit = new DigitalInput(1);

  TalonSRX talon = new TalonSRX(0);
  public Split() {
  }

  public void movingWlimit(double power){
    if(topLimit.get() && power > 0){
      talon.set(ControlMode.PercentOutput, 0);
    }
    else if(bottomLimit.get() && power < 0){
      talon.set(ControlMode.PercentOutput, 0);
    }
    else{
     talon.set(ControlMode.PercentOutput, power);
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
