package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeSS extends SubsystemBase {
    TalonSRX rightIntake = new TalonSRX(Constants.talon3);
    TalonSRX leftIntake = new TalonSRX(Constants.talon4);

  public intakeSS() {
    rightIntake.configFactoryDefault();
    leftIntake.configFactoryDefault();

    rightIntake.setNeutralMode(NeutralMode.Brake);
    leftIntake.setNeutralMode(NeutralMode.Brake);

    leftIntake.follow(rightIntake);

    rightIntake.setInverted(InvertType.InvertMotorOutput);
    leftIntake.setInverted(InvertType.FollowMaster);
  }

  public void inAndOut(Boolean In, Boolean Out){
    if(In) {
        rightIntake.set(ControlMode.PercentOutput, -1);
    }
    else if(Out)
        rightIntake.set(ControlMode.PercentOutput, 1);
    else{
        rightIntake.set(ControlMode.PercentOutput, 0);
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
