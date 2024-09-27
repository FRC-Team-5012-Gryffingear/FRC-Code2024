// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class pneumaticVacuum extends SubsystemBase {
  /** Creates a new pneumaticVacuum. */
  TalonSRX talon = new TalonSRX(Constants.talon1);
  public pneumaticVacuum() {
    talon.configFactoryDefault();
    talon.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void basicMovement(boolean rightTrig, boolean leftTrig){
    if(rightTrig){
        talon.set(ControlMode.PercentOutput, 0.6);
    }
    else if(leftTrig){
        talon.set(ControlMode.PercentOutput, -0.6);
    }
    else{
        talon.set(ControlMode.PercentOutput, 0);
    }
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
