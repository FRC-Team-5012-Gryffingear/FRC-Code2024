// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arcReactor extends SubsystemBase {
  /** Creates a new arcReactor. */
    TalonSRX Motor1 = new TalonSRX(0); //Front Left
    TalonSRX Motor2 = new TalonSRX(1); //Front Right
    TalonSRX Moter3 = new TalonSRX (2); //Back Left
    TalonSRX Moter5 = new TalonSRX (5); //Back Right

    TalonFX e = new TalonFX(10);
    TalonFX eTwo = new TalonFX(2);
    TalonFX r = new TalonFX(20);
    TalonFX rTwo = new TalonFX(4);

    DutyCycleOut e2 = new DutyCycleOut(0);
    DutyCycleOut r2 = new DutyCycleOut(1);

  public arcReactor() {
    // e.getConfigurator().apply(new TalonFXConfiguration());

    // e.setInverted(true);

    // e.setNeutralMode(NeutralModeValue.Brake);

    // e.setControl(new Follower(2, true));

    TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration rightConfiguration = new TalonFXConfiguration();

    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    e.getConfigurator().apply(leftConfiguration);
    eTwo.getConfigurator().apply(leftConfiguration);
    r.getConfigurator().apply(rightConfiguration);
    rTwo.getConfigurator().apply(rightConfiguration);

    eTwo.setControl(new Follower(e.getDeviceID(), false));
    rTwo.setControl(new Follower(e.getDeviceID(), false));

    e.setSafetyEnabled(true);
    r.setSafetyEnabled(true);

    e.setNeutralMode(NeutralModeValue.Brake);
    r.setNeutralMode(NeutralModeValue.Brake);
    eTwo.setNeutralMode(NeutralModeValue.Brake);
    rTwo.setNeutralMode(NeutralModeValue.Brake);

    



    Motor1.configFactoryDefault();
    Motor2.configFactoryDefault();
    Moter3.configFactoryDefault();
    Moter5.configFactoryDefault();

    Motor1.setNeutralMode(NeutralMode.Brake);
    Motor2.setNeutralMode(NeutralMode.Brake);
    Moter3.setNeutralMode(NeutralMode.Brake);
    Moter5.setNeutralMode(NeutralMode.Brake);

    Motor1.follow(Moter3); // left
    Motor2.follow(Moter5); // right 

    Motor2.setInverted(InvertType.InvertMotorOutput);
    Moter5.setInverted(InvertType.InvertMotorOutput);
  }

  public void attemptFXFunction(double fwd, double rot){
    e2.Output = fwd + rot;
    r2.Output = fwd - rot;

    e.setControl(e2);
    r.setControl(r2);
    
  }

  public void silly(double power,double turn){
    Motor1.set(ControlMode.PercentOutput, power + turn);

    Motor2.set(ControlMode.PercentOutput, power - turn);    

    




  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void LED_blink(int x, int y){
    DigitalOutput LED = new DigitalOutput(x);
    Timer.delay(y);
    LED.set(false);
    Timer.delay(y);
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
