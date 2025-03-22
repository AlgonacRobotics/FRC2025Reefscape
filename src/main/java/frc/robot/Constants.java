// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Elevator {
    public static final double elevatorHomePosition = 0; // A
    public static final double elevatorCoralL2 = -17.423828; // B
    public static final double elevatorCoralL3 = -30.526367;// X
    public static final double elevatorCoralHigh = -27;// Y

    public static final int elevatorDrive1_ID = 40;
    public static final int elevatorDrive2_ID = 41;
  }

  public static class Wrist {
    public static final int wristDrive_ID = 42;

    public static final int wristCancoder_ID = 13;

    public static final double wristCollect = 0;  //Dpad down
    public static final double wristCoralL2 = -4.506836;  //Dpad Right
    public static final double wristCoralL3 = -1.572754;  // Dpad Up
    public static final double wristTravel = -5; 
  }

  public static class Intake {
    public static final int intakeDrive_ID = 43;

    public static final int intakeCancoder_ID = 14;

    public static final double intakeOpen = 0.15;  // R bumper
    public static final double intakeClose = 0;  //L bumper
  }
}
