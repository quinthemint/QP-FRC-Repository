/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

   private Joystick joy1 =  new Joystick(0);

   private TalonSRX leftMaster = new TalonSRX(3);
   private TalonSRX rightMaster = new TalonSRX(1);
   private VictorSPX leftSlave = new VictorSPX(1);
   private VictorSPX rightSlave = new VictorSPX(2);

   private TalonSRX armMotor = new TalonSRX(5);
   private VictorSPX armSlave = new VictorSPX(3);

   private TalonSRX rollerMotor = new TalonSRX(4);

   private Compressor compressor = new Compressor();
   private DoubleSolenoid hatchIntake = new DoubleSolenoid(0,1);//PCM Port 0,1
    
   // joysticks
   private Joystick driverJoystick = new Joystick(0);
   private Joystick operatorJoystick = new Joystick(1);

   //unit conversion
   private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
   private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 *18 / 84;

   @Override
   public void robotPeriodic() {
     SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition() * kArmTick2Deg);
     SmartDashboard.putNumber("Left Drive Encoder Value", leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
     SmartDashboard.putNumber("Right Drive Encoder Value", rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);
   }
  @Override
  public void robotInit() {
    // inverted settings
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    armMotor.setInverted(false);

    //slave setups
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    armSlave.follow(armMotor);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    armSlave.setInverted(InvertType.FollowMaster);

    //init encoders
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,10);

    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(true);
    armMotor.setSensorPhase(true);

    // reset encoders to zero
    leftMaster.setSelectedSensorPosition(0,0,10);
    rightMaster.setSelectedSensorPosition(0,0,10);
    armMotor.setSelectedSensorPosition(0,0,10);

      //set encoder boundary limits: to stop motors
      armMotor.configReverseSoftLimitThreshold((int) (0 / kArmTick2Deg), 10);
      armMotor.configForwardSoftLimitThreshold((int) (175 / kArmTick2Deg), 10);

      armMotor.configReverseSoftLimitEnable(true, 10);
      armMotor.configForwardSoftLimitEnable(true, 10);
    //start compressor
    compressor.start();

  }

  @Override
  public void autonomousInit() { 
    enableMotors(true);
    // reset encoders to zero
    leftMaster.setSelectedSensorPosition(0,0,10);
    rightMaster.setSelectedSensorPosition(0,0,10);
    armMotor.setSelectedSensorPosition(0,0,10);
  }

  @Override
  public void autonomousPeriodic() {
    double leftPosition = leftMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double rightPostition = rightMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPostition) / 2;

    if (distance < 10) {
      drive.tankDrive(0.6, 0.6);
    }else{
      drive.tankDrive(0,0);
    }
  }

  @Override
  public void disabledInit() {
    enableMotors(false);
  }
  
  @Override
  public void teleopInit() {
    enableMotors(true);
  }

  @Override
  public void teleopPeriodic() {
  
    //replacement for Differential Drive
     double power = -driverJoystick.getRawAxis(1);
     double speed = -joy1.getRawAxis(1) * 0.6;
     double turn = joy1.getRawAxis(4) * 0.3;
    //deadband
    if (Math.abs(power) <0.05) {
        power = 0;
    }
    if (Math.abs(turn) < 0.05) {
      turn = 0;
    }
    
     double left = speed + turn;
     double right = speed - turn;

     leftMaster.set(ControlMode.PercentOutput, left);
     rightMaster.set(ControlMode.PercentOutput, right);
     leftSlave.set(ControlMode.PercentOutput, -left);
     rightSlave.set(ControlMode.PercentOutput,-right);
   
    //arm control
    double armPower = -operatorJoystick.getRawAxis(1);
    if (Math.abs(armPower) < 0.05) {
      armPower = 0;
    }
    armPower *= 0.5;
    armMotor.set(ControlMode.PercentOutput, armPower);

    //roller control
    double rollerPower = 0;
    if (operatorJoystick.getRawButton(1)== true){
      rollerPower = 1;
    } else if (operatorJoystick.getRawButton(2)) {
      rollerPower = -1;
    }
    rollerMotor.set(ControlMode.PercentOutput, rollerPower);

    //hatch intake
    if (operatorJoystick.getRawButton(3)) {
      hatchIntake.set(Value.kReverse);
    } else {
      hatchIntake.set(Value.kForward);
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  private void enableMotors(boolean on) {
    NeutralMode mode;
    if (on) {
      mode = NeutralMode.Brake;
    }else{
      mode = NeutralMode.Coast;
    }
  leftMaster.setNeutralMode(mode);
  rightMaster.setNeutralMode(mode);
  leftSlave.setNeutralMode(mode); 
  rightSlave.setNeutralMode(mode); 
  armMotor.setNeutralMode(mode); 
  armSlave.setNeutralMode(mode); 
  rollerMotor.setNeutralMode(mode); 
  }
}
