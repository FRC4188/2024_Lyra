// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.TeleDrive;
import frc.robot.subsystems.shooter.Shooter;

/** Add your docs here. */
public class Automation {
    private final Shooter shooter = Shooter.getInstance();

    private Command intake() {
        return new FeedIntake();
    }
    private Command shoot(DoubleSupplier xInput, DoubleSupplier yInput, Translation3d goal) {
        return new ShootOnReady(xInput, yInput, goal);
    }
    private Command drive(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
        return new ParallelCommandGroup(new Stow(), new TeleDrive(xInput, yInput, thetaInput));
    }

    public Command teleOp(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput, Translation3d goal) {
        return intake().andThen(Commands.runOnce(() -> {
                Commands.runOnce(() -> {
                    drive(xInput, yInput, thetaInput).schedule();
                }).andThen(Commands.runOnce(() -> {
                    shoot(xInput, yInput, goal).schedule();
                }).andThen(Commands.runOnce(() -> {
                    teleOp(xInput, yInput, thetaInput, goal).schedule();
                })
                ));
    }));
}
}
