// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSub;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionComm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final VisionSub visionSubsystem;
  // private DigitalOutput LED4 = new DigitalOutput(4);
  // private Timer time = new Timer();

  /**
   * Creates a new VisionComm.
   *
   * @param subsystem The subsystem used by this command.
   */

  // private void LED_on(){
  //   LED4.set(true);
  // }
  // private void LED_off(){
  //   LED4.set(false);
  // }

  // private void LED_blink(){
  //   time.start();
  //   if(time.get() > 0 && time.get() < 0.0625){
  //     LED_on();
  //   }
  //   if(time.get() > 0.25){
  //     LED_off();
  //     if(time.get() > 0.625){
  //       time.reset();
  //     }
  //   }
  // }

  public VisionComm(VisionSub VisSubsystem) {
    visionSubsystem = VisSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(VisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSubsystem.startThread();
    // LED_off();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(visionSubsystem.AprilTagCheck){
    //   LED_blink();
    // }
    // else if(visionSubsystem.Visioncheck){
    //   LED_on();
    // }
    // else {
    //   LED_off();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSubsystem.stopThread();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
