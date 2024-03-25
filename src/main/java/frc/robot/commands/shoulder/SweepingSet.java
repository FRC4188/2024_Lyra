// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoulder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shoulder.Shoulder;

public class SweepingSet extends Command {

  TrapezoidProfile profile;
  State start, goal;
  Timer timer = new Timer();

  /** Creates a new SweepingSet. */
  public SweepingSet(double velocity, double goal) {
    addRequirements(Shoulder.getInstance());
    this.goal = new State(goal, 0.0);
    this.start = new State(Shoulder.getInstance().getAngle().getDegrees(), 0.0);
    profile = new TrapezoidProfile(new Constraints(velocity, Constants.shoulder.MAX_ACCEL));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shoulder.getInstance().setAngle(Rotation2d.fromDegrees(profile.calculate(timer.get(), start, goal).position));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shoulder.getInstance().disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Shoulder.getInstance().getAngle().getDegrees() - goal.position) < Constants.shoulder.ALLOWED_ERROR;
  }
}
