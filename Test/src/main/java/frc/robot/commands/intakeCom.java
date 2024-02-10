package frc.robot.commands;

import frc.robot.subsystems.intakeSS;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class intakeCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final intakeSS Subsystem;
  private final BooleanSupplier Inwards;
  private final BooleanSupplier Outwards;


  public intakeCom(intakeSS subsystem, BooleanSupplier inwards, BooleanSupplier outwards) {
    Subsystem = subsystem;
    Inwards = inwards;
    Outwards = outwards;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Subsystem.inAndOut(Inwards.getAsBoolean(), Outwards.getAsBoolean());
  }
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
