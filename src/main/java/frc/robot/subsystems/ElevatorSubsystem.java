// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  TalonFX elevatorDrive1 = new TalonFX(Constants.Elevator.elevatorDrive1_ID, "rio");
  TalonFX elevatorDrive2 = new TalonFX(Constants.Elevator.elevatorDrive2_ID, "rio");

  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
  //private final PositionVoltage positionControl = new PositionVoltage(0);


  public ElevatorSubsystem() {
    TalonFXConfiguration configEle1 = new TalonFXConfiguration();
    TalonFXConfiguration configEle2 = new TalonFXConfiguration();

    /* Configure gear ratio */
    //FeedbackConfigs fdb = cfg.Feedback;
    //fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

    /* Configure Motion Magic */
    MotionMagicConfigs mmc = configEle1.MotionMagic;
    mmc.withMotionMagicCruiseVelocity(RotationsPerSecond.of(100)) // 5 (mechanism) rotations per second cruise (max 100)
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(75)) // Take approximately 0.5 seconds to reach max vel (max 100)
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk(1600);

    Slot0Configs slot0 = configEle1.Slot0;
    slot0.kS = 0; // Add 0.25 V output to overcome static friction
    slot0.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 10; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.1; // A velocity error of 1 rps results in 0.5 V output

    // position voltage 
   // configEle1.Voltage.withPeakForwardVoltage(8).withPeakReverseVoltage(-8);

    // configure motors
   // configEle1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
   // configEle2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
// if we bring ^ these back uncomment line 64

    //Set Follower
    elevatorDrive2.setControl(new Follower(elevatorDrive1.getDeviceID(), true));

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevatorDrive1.getConfigurator().apply(configEle1);
     // status = elevatorDrive2.getConfigurator().apply(configEle2);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    elevatorDrive1.setPosition(0);
  }

  public void moveToPosition(double rotations){
    elevatorDrive1.setControl(motionMagicControl.withPosition(rotations).withSlot(0));
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
