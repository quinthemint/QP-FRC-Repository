package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;

/**
 * This is a sample program to demonstrate the use of a PIDController with an
 * ultrasonic sensor to reach and maintain a set distance from an object.
 */
public class Robot extends TimedRobot {
  // distance in inches the robot wants to stay from an object
  private static final double kHoldDistance = 12.0;

  // factor to convert sensor values to a distance in inches
  private static final double kValueToInches = 0.125;

  // proportional speed constant
  private static final double kP = 7.0;

  // integral speed constant
  private static final double kI = 0.018;

  // derivative speed constant
  private static final double kD = 1.5;

  private Spark leftMotor1 = new Spark(0);
  private Spark leftMotor2 = new Spark(1);
  private Spark rightMotor1 = new Spark(2);
  private Spark rightMotor2 = new Spark(3);

  private Joystick joy1 = new Joystick(0);

  private AHRS gyro = new AHRS();

  // median filter to discard outliers; filters over 5 samples
  private final MedianFilter m_filter = new MedianFilter(5);

  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  @Override
  public void teleopInit() {
    // Set setpoint of the pid controller
    m_pidController.setSetpoint(kHoldDistance * kValueToInches);
  }

  @Override
  public void teleopPeriodic() {
    // returned value is filtered with a rolling median filter, since ultrasonics
    // tend to be quite noisy and susceptible to sudden outliers
    try {
     
      gyro = new AHRS(SPI.Port.kMXP); 
  } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
  }

  try {
    
    Robot.mecanumDrive_Cartesian(joy1.getX(), joy1.getY(), 
                                currentRotationRate, gyro.getAngle());
} catch( RuntimeException ex ) {
    DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
}

    leftMotor1.set(pidOutput);
    leftMotor2.set(pidOutput);
    rightMotor1.set(pidOutput);
    rightMotor2.set(pidOutput);
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
