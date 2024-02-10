package frc.robot.commands;

import frc.robot.subsystems.elevSS;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class elevCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final elevSS Subsystem;
  private final DoubleSupplier up;
  private final DoubleSupplier down;
  

  /**
   * Creates a new elevCom.
   *
   * @param subsystem The subsystem used by this command.
   */
  public elevCom(elevSS SS, DoubleSupplier Uppp, DoubleSupplier Downnn) {
    Subsystem = SS;
    up = Uppp;
    down = Downnn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Subsystem.upAndDown(up.getAsDouble() - down.getAsDouble());

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

