// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class pneumaticsSubsys extends SubsystemBase {
  /** Creates a new pneumatics. */
  DoubleSolenoid solenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.solenoid1, Constants.solenoid2);
  DoubleSolenoid solenioid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  
  public pneumaticsSubsys() {}

  public void inAndOut(boolean a, boolean b){
    if (a){
      solenoids.set(Value.kForward);
      solenioid2.set(Value.kForward);
    } 
    else if (b){
      solenoids.set(Value.kReverse);
      solenioid2.set(Value.kReverse);
    }
    // else {
    //   solenoids.set(Value.kOff);
    //   solenioid2.set(Value.kOff);
    // }

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
