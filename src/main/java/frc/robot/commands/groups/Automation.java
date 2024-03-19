// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.groups;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import frc.robot.commands.drivetrain.TeleDrive;
// import frc.robot.commands.feeder.FeedIntoFeeder;

// /** Add your docs here. */
// public class Automation {

//     private Command intake(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
//         return new ParallelDeadlineGroup(new FeedIntoFeeder(), new TeleDrive(xInput, yInput, thetaInput));
//     }
//     private Command shoot(DoubleSupplier xInput, DoubleSupplier yInput) {
//         return new ShootOnReady(xInput, yInput, () -> sensors.getMovingShooterRPM(), () -> sensors.getMovingShoulderAngle().getDegrees(), () -> sensors.getMovingDriveAngle().getDegrees());
//     }
//     private Command drive(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
//         return new ParallelCommandGroup(new Stow(), new TeleDrive(xInput, yInput, thetaInput));
//     }

//     public Command teleOp(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
//         return intake(xInput, yInput, thetaInput).andThen(Commands.runOnce(() -> {
//                 Commands.runOnce(() -> {
//                     drive(xInput, yInput, thetaInput).until(() -> sensors.isHappy()).schedule();
//                 }).andThen(Commands.runOnce(() -> {
//                     shoot(xInput, yInput).schedule();
//                 }).andThen(Commands.runOnce(() -> {
//                     teleOp(xInput, yInput, thetaInput).schedule();
//                 })
//                 ));
//     }));
// }
// }