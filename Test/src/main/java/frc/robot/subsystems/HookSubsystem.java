// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSubsystem extends SubsystemBase {
  /** Creates a new HookSubsystem. */
  TalonSRX hookMotor = new TalonSRX(21);
  Timer hookTimer = new Timer();
  
  private Timer time = new Timer();
 private DigitalOutput LED0 = new DigitalOutput(0);

  private boolean hookAttached = false;

  //Red = 0

 // private LedSubsystem ledsubsys = new LedSubsystem();
  
  public HookSubsystem() {
    hookMotor.configFactoryDefault();

    hookMotor.setInverted(InvertType.InvertMotorOutput);
  }

  private void LED_on(){
    LED0.set(true);
  }
  private void LED_off(){
    LED0.set(false);
  }

  private void LED_blink(){
    time.start();
    if(time.get() > 0 && time.get() < 0.0625){
      LED_on();

    }
    if(time.get() > 0.25){
      LED_off();
      if(time.get() > 0.625){
        time.reset();
      }
  }
}

  public void Hooking(boolean Y, boolean X){
    if(Y){
        hookTimer.start();
        if(hookTimer.get() > 1.5){
            hookMotor.set(ControlMode.PercentOutput, 0.5);
            LED_on();
            hookAttached = true;
        }
        else {
          LED_blink();
        }
        
    }
    else if(X){
      hookMotor.set(ControlMode.PercentOutput, -0.25);
      hookTimer.stop();
      hookTimer.reset();
      LED_blink();
    }    
    else{
        hookMotor.set(ControlMode.PercentOutput, 0);
        hookTimer.stop();
        hookTimer.reset();
        if(hookAttached){
          LED_on();
        }
        else{
          LED_off();
        }
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
