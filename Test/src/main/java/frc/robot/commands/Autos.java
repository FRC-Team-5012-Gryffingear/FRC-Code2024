// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSub;

public class Autos extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSub vision;
    private final PIDController xPIDController;
    private final PIDController zPIDController;
    private final PIDController rollPIDController;

    public Autos(SwerveSubsystem swerve, VisionSub vision) {
        this.swerveSubsystem = swerve;
        this.vision = vision;
        addRequirements(swerveSubsystem, vision);

        // Tune these PID constants
        this.xPIDController = new PIDController(0.1, 0, 0); // PID for X offset
        this.zPIDController = new PIDController(0.0001, 0, 0); // PID for distance
        this.rollPIDController = new PIDController(0.05, 0, 0); // For roll (angle) adjustments, tune as necessary
    }
    @Override
    public void initialize(){
      vision.startThread();
    }

    @Override
    public void execute() {
        // if(Math.abs(SmartDashboard.getNumber("ID 5 X Value", 0)) > 0) {
        //     double id_5_Z_offset = vision.getZID(5)/41;

        //     double id_5_X_offset = vision.getXID(5)/61.7;

        //     double id_5_roll_offset = vision.getIDroll(5);

        //     double xAdjustment = 0;

        //     // Calculate PID outputs
        //     if (id_5_X_offset > 0) {
        //       if (id_5_X_offset > 20) {
        //         xAdjustment = xPIDController.calculate(id_5_X_offset, 20); // Target X offset is 0 (centered)
        //       }
        //     }

        //     else {
        //       if (id_5_X_offset < -20) {
        //         xAdjustment = xPIDController.calculate(id_5_X_offset, -20); // Target X offset is 0 (centered)
        //       }
        //     }
            
        //     double zAdjustment = zPIDController.calculate(id_5_Z_offset, 200); // Target distance (e.g., 1 meter)
        //     double rollAdjustment = rollPIDController.calculate(id_5_roll_offset, 0); // Target roll is 0 (aligned)

        //     // Drive robot using calculated adjustments
        //     // Adjust swerve drive call as necessary for your swerve implementation
        //     swerveSubsystem.drive3(xAdjustment, -zAdjustment, rollAdjustment, false);
        // }
        
        // else if(Math.abs(SmartDashboard.getNumber("ID 6 X Value", 0)) > 0) {
        //     double id_6_Z_offset = vision.getZID(6)/41;

        //     double id_6_X_offset = vision.getXID(6)/61.7;

        //     double id_6_roll_offset = vision.getIDroll(6) - .1;

        //     // Calculate PID outputs
        //     double xAdjustment = 0;
            
        //     // Calculate PID outputs
        //     if (id_6_X_offset > 0) {
        //       if (id_6_X_offset > 20) {
        //         xAdjustment = xPIDController.calculate(id_6_X_offset, 20); // Target X offset is 0 (centered)
        //       }
        //     }

        //     else {
        //       if (id_6_X_offset < -20) {
        //         xAdjustment = xPIDController.calculate(id_6_X_offset, -20); // Target X offset is 0 (centered)
        //       }
        //     }
        //     double zAdjustment = zPIDController.calculate(id_6_Z_offset, 83); // Target distance (e.g., 1 meter)
        //     double rollAdjustment = rollPIDController.calculate(id_6_roll_offset, 0); // Target roll is 0 (aligned)

        //     // Drive robot using calculated adjustments
        //     // Adjust swerve drive call as necessary for your swerve implementation
        //     swerveSubsystem.drive3(xAdjustment, -zAdjustment, rollAdjustment, false);
        // }
        
        // else {
        //     // Stop the robot if no April Tag is detected
        //     swerveSubsystem.stopMods();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopMods();
        vision.stopThread();
    }

    @Override
    public boolean isFinished() {
        // Optionally, end command when close enough to April Tag
        // Define "close enough" as per your requirements
        return false;
    }
}

