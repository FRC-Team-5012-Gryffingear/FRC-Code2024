// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class practiceArcade extends SubsystemBase {
  /** Creates a new practiceArcade. */
  TalonSRX talonTR = new TalonSRX(Constants.mtalon1);
  TalonSRX talonTL = new TalonSRX(Constants.mtalon2);
  TalonSRX talonBR = new TalonSRX(Constants.mtalon3);
  TalonSRX talonBL = new TalonSRX(Constants.mtalon4);
  

  public practiceArcade() {
    talonTR.configFactoryDefault();
    talonTL.configFactoryDefault();
    talonBR.configFactoryDefault();
    talonBL.configFactoryDefault();

    talonTR.setNeutralMode(NeutralMode.Coast);
    talonTL.setNeutralMode(NeutralMode.Coast);
    talonBR.setNeutralMode(NeutralMode.Coast);
    talonBL.setNeutralMode(NeutralMode.Coast);

    talonBR.follow(talonTR);
    talonBL.follow(talonTL);

    talonTR.setInverted(InvertType.InvertMotorOutput);
    talonBR.setInverted(InvertType.FollowMaster);

  }



  public void move_and_turn(double power, double turn){
    talonTR.set(ControlMode.PercentOutput, power - turn);
    talonTL.set(ControlMode.PercentOutput, power + turn);
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
