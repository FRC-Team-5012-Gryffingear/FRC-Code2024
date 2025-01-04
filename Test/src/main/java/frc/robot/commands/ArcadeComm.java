// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArcadeSubsys;
import frc.robot.subsystems.limey;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArcadeComm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArcadeSubsys arcade;

  private final DoubleSupplier RT, LT, Tu;

  private final BooleanSupplier Zlock, RotLock;

  private final limey lim = new limey();

  /**
   * Creates a new ArcadeComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeComm(ArcadeSubsys subsystem, DoubleSupplier Ri, DoubleSupplier Le, DoubleSupplier T, BooleanSupplier Zlock, BooleanSupplier RotLock) {
    arcade = subsystem;
    RT = Ri;
    LT = Le;
    Tu = T;
    this.Zlock = Zlock;
    this.RotLock = RotLock;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arcade.move_turn(RT.getAsDouble() - LT.getAsDouble(), Tu.getAsDouble());



        arcade.move_turn(lim.fwrdLock(lim.estimate3DZInches()), 0);
    
        arcade.move_turn(0, lim.rotationLock(lim.getX()));
        lim.rotAround(lim.getX());

    arcade.button(Zlock.getAsBoolean());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
