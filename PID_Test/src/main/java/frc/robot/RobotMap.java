/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap {

  // Speed Constants
  public static final double DRIVE_SPEED = 0.4;
  public static final double TURN_SPEED = 0.4;

  // Drivetrain ports (PWM)
  public static final int LEFT_FRONT = 0;
  public static final int LEFT_BACK = 1;
  public static final int RIGHT_FRONT = 2;
  public static final int RIGHT_BACK = 3;

  // Joystick ports & all xbox raw values
  public static final int DRIVER_CONTROLLER = 0;
  public static final int OPERATOR_CONTROLLER = 1;
  public static final int DRIVETRAIN_JOYSTICK_X = 0;
  public static final int DRIVETRAIN_JOYSTICK_Y = 1;
  public static final int DRIVER_A_BUTTON = 1;
}
