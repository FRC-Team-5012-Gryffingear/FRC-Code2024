// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new LedSubsystem. */
// subject to change ports
  // private DigitalOutput LED0 = new DigitalOutput(0);
  // private DigitalOutput LED1 = new DigitalOutput(1);
  // private DigitalOutput LED2 = new DigitalOutput(2);
  // private DigitalOutput LED3 = new DigitalOutput(3);
  // private DigitalOutput LED4 = new DigitalOutput(4);

  private Timer time = new Timer();
  public LedSubsystem() {

  }
   public void LED_on(int port) {
    // LED4.set(true);
 }
  public void LED_off(int port) {
    // LED4.set(false);
  }


  public void LED_blink(int port){
    time.start();
    if(time.get() > 0 && time.get() < 0.0625){
      LED_on(port);
      // if(time.get() > 1.5){
      //   LED_off(port);
      //   if(time.get() > 2){
      //     time.reset();
      //   }
      // }
    }
    if(time.get() > 0.25){
      LED_off(port);
      if(time.get() > 0.625){
        time.reset();
      }
    }
    // LED_on(port); //turn on LED
    // Timer.delay(duration); //wait for specified duration
    // LED_off(port); //turns off LED
    // Timer.delay(duration); //wait for specified duration
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
