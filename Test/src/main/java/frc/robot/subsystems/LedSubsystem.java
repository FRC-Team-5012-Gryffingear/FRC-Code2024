// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new LedSubsystem. */

  DigitalInput LED1;
  DigitalInput LED2;
  DigitalInput LED3;
  DigitalInput LED4;
  DigitalInput LED5;
  public LedSubsystem() {

  }
  public void LED_on(int port) {
    if(port == 1){
        LED1 = new DigitalInput(1);
    }
    else if(port == 2){
        LED2 = new DigitalInput(2);
    }
    else if(port == 3){
        LED3 = new DigitalInput(3);
    }    
    else if(port == 4){
        LED4 = new DigitalInput(4);
    }
    else if(port == 5){
        LED5 = new DigitalInput(5);
    }
  }
  public void LED_off(int port) {
    if(port == 1){
        LED1.close();
    }
    else if(port == 2){
        LED2.close();
    }
    else if(port == 3){
        LED3.close();
    }    
    else if(port == 4){
        LED4.close();
        }
    else if(port == 5){
        LED5.close();
    }
  }
  public void LED_blink(int port, double duration){
    LED_on(port); //turn on LED
    Timer.delay(duration); //wait for specified duration
    LED_off(port); //turns off LED
    Timer.delay(duration); //wait for specified duration
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
