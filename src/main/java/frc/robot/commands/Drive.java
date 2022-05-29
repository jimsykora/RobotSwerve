// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_DualActionStick;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
  SN_DualActionStick controller;
  Drivetrain drivetrain;

  boolean fieldRelative;

  /** Creates a new Drive. */
  public Drive(Drivetrain a_drivetrain, SN_DualActionStick a_controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = a_drivetrain;
    controller = a_controller;

    fieldRelative = false;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (controller.btn_A.get()) {
      fieldRelative = true;
    }

    if (controller.btn_B.get()) {
      fieldRelative = false;
    }

    drivetrain.drive(controller.getLeftStickX(), controller.getLeftStickY(), controller.getRightStickX(),
        fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
