/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  private TalonSRX armMotor = new TalonSRX(0);
  private VictorSPX armSlave = new VictorSPX(0);

   private Joystick joy1 = new Joystick (0);

   private Encoder encoder = new Encoder (0,1,false, EncodingType.k4X);
    private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 *18 / 84;

  @Override
  public void robotInit() {

    //Setup Slave
    armSlave.follow(armMotor);
    armSlave.setInverted(InvertType.FollowMaster);

    //Setup Encoders
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,10);
    armMotor.setSensorPhase(true);

    //Reseting encoders
    armMotor.setSelectedSensorPosition(0,0,10);

    //Arm limits

     armMotor.configReverseSoftLimitThreshold((int) (0 / kArmTick2Deg), 10);
     armMotor.configForwardSoftLimitThreshold((int) (175 / kArmTick2Deg), 10);

     armMotor.configReverseSoftLimitEnable(true, 10);
     armMotor.configForwardSoftLimitEnable(true, 10);
  }

  @Override
  public void autonomousInit() {
    encoder.reset();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
  }
  //PID constants
  final double kP = 0.5;
  final double kI = 0.5;
  final double kD = 0.1;
  final double iLimit = 1;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  @Override
  public void autonomousPeriodic() {
    encoder.reset();
    // get joystick command
    if (joy1.getRawButton(1)) {
      setpoint = 10;
    }else if (joy1.getRawButton(2));
    setpoint = 0;

    //get sensor position
    double sensorPosition = encoder.get() * kArmTick2Deg;

    //calculations
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

      if (Math.abs(error) < iLimit) {
        errorSum += error * dt;
      }
   
      double errorRate = (error - lastError) / dt;
    
    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

      //output to motors
      armMotor.set(ControlMode.PercentOutput,outputSpeed);

      //update last- variables
      lastTimestamp = Timer.getFPGATimestamp();
      lastError = error;
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get() * kArmTick2Deg);

  }
  

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}