// package frc.robot.commands.climber;

// import frc.robot.Constants;
// import frc.robot.subsystems.climber.Climber;
// import edu.wpi.first.wpilibj2.command.Command;

// public class LowerClimber extends Command {
//   private Climber climber = Climber.getInstance();

//   /** Creates a new SetClimberHeight. */
//   public LowerClimber() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(climber);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//       climber.setLeft(-0.8);
//       climber.setRight(-0.8);
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//       climber.setLeft(0.0);
//       climber.setRight(0.0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }