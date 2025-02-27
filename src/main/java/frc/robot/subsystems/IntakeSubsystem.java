// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  TalonFXS intakeDrive = new TalonFXS(Constants.Intake.intakeDrive_ID, "Canivore1");

  //CANcoder intakeCancoder = new CANcoder(Constants.Intake.intakeCancoder_ID);

  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

  public IntakeSubsystem() {
    TalonFXSConfiguration intakeConfig = new TalonFXSConfiguration();

    intakeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    //intakeConfig.Feedback.FeedbackRemoteSensorID = intakeCancoder.getDeviceID();
    //intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    /* Configure gear ratio */
    //FeedbackConfigs fdb = cfg.Feedback;
    //fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

    /* Configure Motion Magic */
    MotionMagicConfigs mmcIntake = intakeConfig.MotionMagic;
    mmcIntake.withMotionMagicCruiseVelocity(RotationsPerSecond.of(60)) // 5 (mechanism) rotations per second cruise (max 120 for minion)
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(60)) // Take approximately 0.5 seconds to reach max vel (max 120 for minion)
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk(1600);

    Slot0Configs slot0 = intakeConfig.Slot0;
    slot0.kS = 0; // Add 0.25 V output to overcome static friction
    slot0.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 1; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeDrive.getConfigurator().apply(intakeConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    intakeDrive.setPosition(0);

  }

  public void moveToPosition(double rotations){
    intakeDrive.setControl(motionMagicControl.withPosition(rotations).withSlot(0));
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
