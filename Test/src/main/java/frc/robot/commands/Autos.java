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

