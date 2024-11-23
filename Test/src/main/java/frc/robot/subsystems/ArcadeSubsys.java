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

public class ArcadeSubsys extends SubsystemBase {
  /** Creates a new ArcadeSubsys. */
  TalonSRX talonFL = new TalonSRX(Constants.FL);
  TalonSRX talonFR = new TalonSRX(Constants.FR);
  TalonSRX talonBL = new TalonSRX(Constants.BR);
  TalonSRX talonBR = new TalonSRX(Constants.BL);

  public ArcadeSubsys() {
    talonFL.configFactoryDefault();
    talonFR.configFactoryDefault();
    talonBL.configFactoryDefault();
    talonBR.configFactoryDefault();

    talonFL.setNeutralMode(NeutralMode.Brake);
    talonFR.setNeutralMode(NeutralMode.Brake);
    talonBL.setNeutralMode(NeutralMode.Brake);
    talonBR.setNeutralMode(NeutralMode.Brake);

    talonBL.follow(talonFL);
    talonBR.follow(talonFR);

    talonFR.setInverted(InvertType.InvertMotorOutput);
    talonBR.setInverted(InvertType.FollowMaster);

  }

  public void move_turn(double power, double turn){
    talonFL.set(ControlMode.PercentOutput, power + turn);
    talonFR.set(ControlMode.PercentOutput, power - turn);
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
