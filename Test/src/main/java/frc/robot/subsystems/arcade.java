// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arcade extends SubsystemBase {
  /** Creates a new arcade. */
  TalonSRX drive1 = new TalonSRX(Constants.FL); //LeftFront
  TalonSRX drive2 = new TalonSRX(Constants.FR); //RightFront
  TalonSRX drive3 = new TalonSRX(Constants.BL); //LeftBack
  TalonSRX drive4 = new TalonSRX(Constants.BR); //RightBack
  
  public arcade() {
    // do the same configurations for drive 3 and 4
    drive1.configFactoryDefault();
    drive2.configFactoryDefault();
    drive3.configFactoryDefault();
    drive4.configFactoryDefault();

    // do the same configurations for drive 3 and 4
    drive1.setNeutralMode(NeutralMode.Brake);
    drive2.setNeutralMode(NeutralMode.Brake);
    drive3.setNeutralMode(NeutralMode.Brake);
    drive4.setNeutralMode(NeutralMode.Brake);

    drive3.follow(drive1);
    drive4.follow(drive2);

    drive2.setInverted(InvertType.InvertMotorOutput);
    drive4.setInverted(InvertType.FollowMaster);


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
    public void moveandturn(double power, double turn){
        drive1.set(ControlMode.PercentOutput, power + turn);
        drive2.set(ControlMode.PercentOutput, power - turn);
    }

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
