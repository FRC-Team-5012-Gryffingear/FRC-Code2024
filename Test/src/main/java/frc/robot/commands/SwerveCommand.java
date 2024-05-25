// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSub;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class SwerveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 private final SwerveSubsystem swervesubsys;
 private final CommandXboxController controller2;
  private final BooleanSupplier yaw;
  //private final LedSubsystem Ledsubsys; 
  public double t = 1;
  private VisionSub vision = new VisionSub();

 
  
  public SwerveCommand(SwerveSubsystem subsystem, CommandXboxController controller,BooleanSupplier yaw) {
    swervesubsys = subsystem;
    controller2 = controller;
    this.yaw = yaw;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.startThread();
    swervesubsys.resetHeading();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = MathUtil.applyDeadband(controller2.getLeftY(), OperatorConstants.DriverDeadband) * 0.4;
    double ySpeed = MathUtil.applyDeadband(controller2.getLeftX(), OperatorConstants.DriverDeadband) * 0.4;
    double rotateSpeed = MathUtil.applyDeadband(controller2.getRightX(), OperatorConstants.DriverDeadband) * 0.4;
    

    if(Math.abs(xSpeed) < 0.02 && Math.abs(ySpeed) < 0.02){
      t = 1; 
    }

    double numeratorX =Math.pow(xSpeed, t/1000);
    double denomX = Math.pow(2, t/1000);

    double numeratorY =Math.pow(ySpeed, t/1000);
    double denomY = Math.pow(2, t/1000);

    double alpha = 0.015; //changed from .015

    double finalPushX = 1 - Math.pow(Math.E, -alpha*t*xSpeed);
    double finalPushY = 1 - Math.pow(Math.E, -alpha*t*ySpeed);

    if(finalPushX > 1){ //changed, used to be finalPushX > 1
      finalPushX = 1;
    }
    else if(finalPushX < -1){
      finalPushX = -1;
    }


    if(finalPushY > 1){
      finalPushY = 1;
    }
    else if(finalPushY < -1){
      finalPushY = -1;
    }

    // if(x < 1000){
    //   finalPushX = (numeratorX/denomX);
    //   finalPushY = (numeratorY/denomY);
    // }
    // else{
    //   finalPushX = 1 - (numeratorX/denomX);
    //   finalPushY = 1 - (numeratorY/denomY);
    // }
    System.out.println("This is final PUSHX " + finalPushX);
    System.out.println("This is final PUSH Y " + finalPushY);
    System.out.println("ORIGINAL X VAL " + xSpeed);
    System.out.println("ORIGINAL Y VAL " + ySpeed);
    System.out.println("THIS IS X VAL  " + t);
    t = t+1;

    // Delete 2 if slow
//X should move pos direction, -Y  move pos direction, rotateSpeed move in pos direction
    // swervesubsys.drive3(finalPushX, -finalPushY, rotateSpeed*1.5, true);
    swervesubsys.drive3(xSpeed, -ySpeed, rotateSpeed*1.5, true);

    
    if(yaw.getAsBoolean()){
      swervesubsys.resetHeading();
    }
    // else if(pose.getAsBoolean()){
    //   swervesubsys.resetPose();
    // }
    // if(controller2.rightBumper().getAsBoolean()){
    //     swervesubsys.drive(xSpeed, ySpeed, rotateSpeed,true);
    // } else{
    //     swervesubsys.drive(xSpeed, ySpeed, rotateSpeed);
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swervesubsys.stopMods();
    vision.stopThread();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
