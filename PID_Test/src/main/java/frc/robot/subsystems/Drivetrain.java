/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.*;

public class Drivetrain extends Subsystem implements PIDOutput {

  private TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;

  public final PIDController turnController;
  private final AHRS ahrs;

  private final double kP = 0;
  private final double kI = 0;
  private final double kD = 0;

  public Drivetrain() {
    leftMaster = new TalonSRX(RobotMap.LEFT_FRONT);
    leftSlave = new TalonSRX(RobotMap.LEFT_BACK);
    rightMaster = new TalonSRX(RobotMap.RIGHT_FRONT);
    rightSlave = new TalonSRX(RobotMap.RIGHT_BACK);
    ahrs = new AHRS(SPI.Port.kMXP);

    configTalon(leftMaster);
    configTalon(leftSlave);
    configTalon(rightMaster);
    configTalon(rightSlave);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    turnController = new PIDController(kP, kI, kD, ahrs, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-0.45, 0.45);
    turnController.setAbsoluteTolerance(2.0f);
    turnController.setContinuous();
  }

  public void set(ControlMode mode, double leftSpeed, double rightSpeed) {
    leftMaster.set(mode, leftSpeed);
    rightMaster.set(mode, rightSpeed);
  }

  public void rotateDegrees(double angle) {
    ahrs.reset();
    turnController.reset();
    turnController.setPID(kP, kI, kD);
    turnController.setSetpoint(angle);
    turnController.enable();
  }

  public static void configTalon(TalonSRX motor) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.neutralOutput();
    motor.setSensorPhase(false);
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);
    motor.configClosedloopRamp(0.5, 0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDrive());
  }

  @Override
  public void pidWrite(double output) {
    set(ControlMode.PercentOutput, -output, output);
  }
}
