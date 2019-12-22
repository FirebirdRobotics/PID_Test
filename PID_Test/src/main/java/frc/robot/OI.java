/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

    public static final double JOY_DEADZONE = 0.05;

    public final XboxController driverController;
    public final XboxController operatorController;

    public final Button quickTurnCheesyDrive;

    public OI() {
        driverController = new XboxController(RobotMap.DRIVER_CONTROLLER);
        operatorController = new XboxController(RobotMap.OPERATOR_CONTROLLER);

        quickTurnCheesyDrive = new JoystickButton(driverController, RobotMap.DRIVER_A_BUTTON);
    }

    public double getDrivetrainX() {
        double raw = driverController.getRawAxis(RobotMap.DRIVETRAIN_JOYSTICK_X);
        return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
    }

    public double getDrivetrainY() {
        double raw = driverController.getRawAxis(RobotMap.DRIVETRAIN_JOYSTICK_Y);
        return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
    }
}
