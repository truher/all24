// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import edu.wpi.first.wpilibj2.command.Command;

public class Empty extends Command {
  /** Creates a new Empty. */
  public Empty() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("asdf");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("AHHHHHHHHHH");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("asdasafsd");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}