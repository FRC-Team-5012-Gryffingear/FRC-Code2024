package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class elevSS extends SubsystemBase {
    TalonSRX rightTalon = new TalonSRX(Constants.talon1);
    TalonSRX leftTalon = new TalonSRX(Constants.talon2);
  public elevSS() {
    rightTalon.configFactoryDefault();
    leftTalon.configFactoryDefault();

    rightTalon.setNeutralMode(NeutralMode.Coast);
    leftTalon.setNeutralMode(NeutralMode.Coast);

    leftTalon.follow(rightTalon);

    rightTalon.setInverted(InvertType.InvertMotorOutput);
    
  }

  public void upAndDown(double Up) {
    rightTalon.set(ControlMode.PercentOutput, Up);
    leftTalon.set(ControlMode.PercentOutput, Up);
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

