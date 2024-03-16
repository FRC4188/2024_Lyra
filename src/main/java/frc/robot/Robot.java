// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import CSP_Lib.inputs.CSP_Controller;
import CSP_Lib.utils.DashboardManager;
import CSP_Lib.utils.TempManager;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.feeder.Feeder;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private CSP_Controller pilot = new CSP_Controller(Constants.io.PILOT_CONTROLLER);
  private CSP_Controller copilot = new CSP_Controller(Constants.io.COPILOT_CONTROLLER);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer(pilot, copilot);
    addPeriodic(() -> TempManager.monitor(), 2.0);

    DashboardManager.start(0.01);

    DashboardManager.addNumberInput("Input A", 1.0);
    DashboardManager.putValue("Output B", 2.0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    DashboardManager.putValue("Input A progress", DashboardManager.readNumberInput("Input A"));
    DashboardManager.putValue("Output B", RobotController.getFPGATime());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}