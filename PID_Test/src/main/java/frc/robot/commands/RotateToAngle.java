/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RotateToAngle extends Command {

  double Angle;
  boolean isFinished = false;
  boolean inAcceptableErrorRange = false;
  int count;

  public RotateToAngle(double angle) {
    requires(Robot.drivetrain);
    Angle = angle;
  }

  @Override
  protected void initialize() {
    Robot.drivetrain.rotateDegrees(Angle);
  }

  @Override
  protected void execute() {
    double error = Robot.drivetrain.turnController.getError();
    inAcceptableErrorRange = Math.abs(error) < 2;

    if (inAcceptableErrorRange) {
      count++;
      isFinished = count >= 5;
    } else {
      count = 0;
    }
  }

  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  @Override
  protected void end() {
    Robot.drivetrain.turnController.disable();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
