package frc.robot;

import com.kauailabs.navx.frc.AHRS;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

/**
 * This is a sample program to demonstrate the use of a PIDController with an
 * ultrasonic sensor to reach and maintain a set distance from an object.
 */
public class Robot extends TimedRobot {

  // proportional speed constant
  private static final double kP = 0.008;

  // integral speed constant
  private static final double kI = 0.008;

  // derivative speed constant
  private static final double kD = 0.0008;

  private Spark leftMotor1 = new Spark(0);
  private Spark leftMotor2 = new Spark(1);
  private Spark rightMotor1 = new Spark(2);
  private Spark rightMotor2 = new Spark(3);

  private Joystick joy1 = new Joystick(0);

  private AHRS gyro;

  private final PIDController pidController = new PIDController(kP, kI, kD);

  @Override
  public void teleopInit() {

     }

  @Override
  public void teleopPeriodic() {
   
    if (joy1.getRawButton(1)==true){
      pidController.setSetpoint(90);
    }

    else if (joy1.getRawButton(2)==true){
      pidController.setSetpoint(180);
    }

    else if (joy1.getRawButton(3)==true){
      pidController.setSetpoint(270);
    }

    else if (joy1.getRawButton(4)==true){
      pidController.setSetpoint(360);
    }
    
    double pidOutput = pidController.calculate(gyro.getAngle());

    leftMotor1.set(pidOutput);
    leftMotor2.set(pidOutput);
    rightMotor1.set(pidOutput);
    rightMotor2.set(pidOutput);


    SmartDashboard.putNumber("pidOutout in %", pidOutput*100);
    
  }

  @Override
  public void robotInit() {

    
    try {
     
      gyro = new AHRS(SPI.Port.kMXP); 
  } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
  }

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("gyroOutput in angle", gyro.getAngle());
  }

}
