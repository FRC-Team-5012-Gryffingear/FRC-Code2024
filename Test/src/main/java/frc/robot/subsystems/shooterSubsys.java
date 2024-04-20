package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooterSubsys extends SubsystemBase {
    public final TalonSRX shooterTalon1 = new TalonSRX(Constants.talonS1);
    public final TalonSRX shooterTalon2 = new TalonSRX(Constants.talonS2);
  public shooterSubsys() {

    shooterTalon1.configFactoryDefault();
    shooterTalon2.configFactoryDefault();

    shooterTalon1.setNeutralMode(NeutralMode.Brake);
    shooterTalon2.setNeutralMode(NeutralMode.Brake);

  }

  //Uses boolean shootA and shootB as button placeholders and checks if they are pressed. 
  //If they are, then there respectivemotor will move.
  public void shoot(boolean shootA, boolean shootB){
    if(shootA){
        shooterTalon1.set(ControlMode.PercentOutput, 1);
    }
    else{
        shooterTalon1.set(ControlMode.PercentOutput, 0);

    }
    if(shootB){
        shooterTalon2.set(ControlMode.PercentOutput, 1);
    }
    else{
        shooterTalon2.set(ControlMode.PercentOutput, 0);
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
