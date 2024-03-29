// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSubsystem extends SubsystemBase {
  /** Creates a new HookSubsystem. */
  TalonSRX hookMotor = new TalonSRX(21);
  Timer hookTimer = new Timer();
  
  public HookSubsystem() {
    hookMotor.configFactoryDefault();

    hookMotor.setInverted(InvertType.InvertMotorOutput);
  }


  public void Hooking(boolean Y, boolean X){
    if(Y){
        hookTimer.start();
        if(hookTimer.get() > 1.5){
            hookMotor.set(ControlMode.PercentOutput, 0.5);
        }
    }
    else if(X){
      hookMotor.set(ControlMode.PercentOutput, -0.25);
      hookTimer.stop();
      hookTimer.reset();
    }    
    else{
        hookMotor.set(ControlMode.PercentOutput, 0);
        hookTimer.stop();
        hookTimer.reset();
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
