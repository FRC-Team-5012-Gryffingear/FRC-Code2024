// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class arcadeSubsys extends SubsystemBase {
  /** Creates a new arcadeSubsys. */
  TalonSRX talonFL = new TalonSRX(Constants.talon1);
  TalonSRX talonFR = new TalonSRX(Constants.talon2);
  TalonSRX talonBL = new TalonSRX(Constants.talon3);
  TalonSRX talonBR = new TalonSRX(Constants.talon4);

  Pigeon2 pigeon = new Pigeon2(Constants.pigeonport);

  public arcadeSubsys() {
    talonFL.configFactoryDefault();
    talonFR.configFactoryDefault();
    talonBL.configFactoryDefault();
    talonBR.configFactoryDefault();

    talonFL.setNeutralMode(NeutralMode.Coast);
    talonFR.setNeutralMode(NeutralMode.Coast);
    talonBL.setNeutralMode(NeutralMode.Coast);
    talonBR.setNeutralMode(NeutralMode.Coast);

    talonBL.follow(talonFL);
    talonBR.follow(talonFR);

    talonFR.setInverted(InvertType.InvertMotorOutput);
    talonBR.setInverted(InvertType.FollowMaster);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  //Motors are run by power, + or - turn to control the turning. 
  public void motionTurning(double power, double turn){
    talonFR.set(ControlMode.PercentOutput, power - turn);
    talonFL.set(ControlMode.PercentOutput, power + turn);
  }
  //Gets the angle of the pigeon in degrees
  public Rotation2d angle(){
    return Rotation2d.fromDegrees(pigeon.getAngle());
  }

  //As stated, it resets the yaw back to 0.
  public void yawReset(){
    pigeon.reset();
    pigeon.setYaw(0);
  }


  public void autocorrect(){
    
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
