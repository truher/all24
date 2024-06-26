// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Servos;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
  public void robotPeriodic() {
   // System.out.println("Lower arm" + m_exampleSubsystem.getEncoder(Servos.LOWER_ARM));
   // System.out.println("Upper arm" + m_exampleSubsystem.getEncoder(Servos.UPPER_ARM));
  }
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.nDriverControllerPort);
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.rightTrigger().whileTrue(m_exampleSubsystem.exampleMethodCommand(0.01));
    m_driverController.leftTrigger().whileTrue(m_exampleSubsystem.exampleMethodCommand(-0.01));
    m_driverController.a().whileTrue(m_exampleSubsystem.MyMethodCommand(-0.01));
    m_driverController.y().whileTrue(m_exampleSubsystem.YourMethodCommand(0.01));
    m_driverController.b().whileTrue(m_exampleSubsystem.YourMethodCommand(-0.01));
    m_driverController.rightBumper().whileTrue(m_exampleSubsystem.HisMethodCommand(0.01));
    m_driverController.leftBumper().whileTrue(m_exampleSubsystem.HisMethodCommand(-0.01));
    m_driverController.povRight().whileTrue(m_exampleSubsystem.HerMethodCommand(0.01));
    m_driverController.povLeft().whileTrue(m_exampleSubsystem.HerMethodCommand(-0.01));
    m_operatorController.a().whileTrue(m_exampleSubsystem.TheirMethodCommand(0.01));
    m_operatorController.b().whileTrue(m_exampleSubsystem.TheirMethodCommand(-0.01));
    m_operatorController.rightTrigger().whileTrue(m_exampleSubsystem.exampleMethodCommand(0.01));
    m_operatorController.leftTrigger().whileTrue(m_exampleSubsystem.exampleMethodCommand(-0.01));
    m_operatorController.back().whileTrue(m_exampleSubsystem.HerMethodCommand(0.01));
    m_operatorController.start().whileTrue(m_exampleSubsystem.HerMethodCommand(-0.01));
    m_operatorController.rightBumper().whileTrue(m_exampleSubsystem.HisMethodCommand(0.01));
    m_operatorController.leftBumper().whileTrue(m_exampleSubsystem.HisMethodCommand(-0.01));
    m_operatorController.povDown().whileTrue(m_exampleSubsystem.BestMethodCommand());
    m_driverController.x().whileTrue(m_exampleSubsystem.setCartesian(new Translation2d(-0.1,0.1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
