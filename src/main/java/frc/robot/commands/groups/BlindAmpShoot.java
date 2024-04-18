// package frc.robot.commands.groups;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// import frc.robot.commands.feeder.FeedIntoShooter;
// import frc.robot.commands.shoulder.SetShoulderAngle;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.shooter.Shooter.ControlMode;
// import frc.robot.subsystems.shoulder.Shoulder;

// public class BlindAmpShoot extends ParallelCommandGroup {
//     Shoulder shoulder = Shoulder.getInstance();
//     Shooter shooter = Shooter.getInstance();

//     public BlindAmpShoot() {
//         addCommands(
//             new SequentialCommandGroup(
//                 new ParallelCommandGroup(
//                     new SetShoulderAngle(() -> 10.0),
//                     new RunCommand(() -> shooter.setVelocity(5.0)),
//                                         new RunCommand(() -> shooter.setControlMode(ControlMode.VELOCITY))

//                 ).until(() -> shooter.atMPS(3.0, 0.5) && shoulder.atGoal(Rotation2d.fromDegrees(10.0), 1.5)),
//                 new ParallelCommandGroup(
//                     Commands.waitSeconds(0.2).andThen(new FeedIntoShooter(12.0)),
//                     new RunCommand(() -> shooter.setVelocity(5.0)),
//                     new SetShoulderAngle(() -> 40.0)
//                 )
//             )
//         );
//     }
// }

// package frc.robot.commands.groups;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// import frc.robot.commands.feeder.FeedIntoShooter;
// import frc.robot.commands.shooter.SetShooterMPS;
// import frc.robot.commands.shoulder.SetShoulderAngle;
// import frc.robot.commands.shoulder.SweepingSet;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.shoulder.Shoulder;

// public class BlindAmpShoot extends ParallelCommandGroup {
//     Shoulder shoulder = Shoulder.getInstance();
//     Shooter shooter = Shooter.getInstance();

//     public BlindAmpShoot() {
//         addCommands(
//             new SequentialCommandGroup(
//                 new ParallelCommandGroup(
//                     new SetShoulderAngle(() -> 10.0),
//                     new SetShooterMPS(() -> 4.0)
//                 ).until(() -> shooter.atMPS() && shoulder.atGoal(10.0)),
//                 new ParallelCommandGroup(
//                     Commands.waitSeconds(0.0).andThen(new FeedIntoShooter(12.0)),
//                     new SetShooterMPS(() -> 4.0),
//                     new SweepingSet(50.0, 40.0)
//                 )
//             )
//         );
//     }
// }

package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.FeedIntoShooter;
import frc.robot.commands.shooter.SetShooterMPS;
import frc.robot.commands.shoulder.SetShoulderAngle;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shoulder.Shoulder;

public class BlindAmpShoot extends SequentialCommandGroup {

    public BlindAmpShoot() {
        addCommands(        
            new ParallelDeadlineGroup(
                Commands.waitUntil(() -> 
                    Shooter.getInstance().atMPS(2.0) && 
                    Shoulder.getInstance().atGoal(Rotation2d.fromDegrees(35.0), 1.0)).andThen(
                new FeedIntoShooter(8.0).andThen(Commands.waitSeconds(0.5))),
                new SetShooterMPS(() -> 3.2),
                new SetShoulderAngle(() -> 35.0)
            )
        );
    }
}