// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
//private RelativeEncoder m_encoder;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;



public class Robot extends TimedRobot {


  // --------------------------------------------------
  // ------------------- VARIABLES --------------------
  // --------------------------------------------------

  private PS4Controller Controller1 = new PS4Controller(0);
  //private PS4Controller Controller2 = new PS4Controller(2);
  
  private CANSparkMax Motor1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax Motor2 = new CANSparkMax(2, MotorType.kBrushless);
  //private CANSparkMax Motor3 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax Motor4 = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax Motor5 = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax Motor6 = new CANSparkMax(6, MotorType.kBrushed);

  private MotorControllerGroup Left = new MotorControllerGroup(Motor2);
  private MotorControllerGroup Right = new MotorControllerGroup(Motor1);
  DifferentialDrive RobotDrive = new DifferentialDrive(Left, Right);
 
  private double startTime;
  // -------------------------------------------------
  // --------------------- ROBOT ---------------------
  // -------------------------------------------------
  private RelativeEncoder m_encoder;
  @Override
  public void robotInit() {
    Motor1.restoreFactoryDefaults();
    Motor2.restoreFactoryDefaults();
    //Motor3.restoreFactoryDefaults();
    Motor4.restoreFactoryDefaults();

    Motor6.setIdleMode(IdleMode.kBrake);

    CameraServer.startAutomaticCapture();
        /**
    * In order to read encoder values an encoder object is created using the 
    * getEncoder() method from an existing CANSparkMax object
    */
    m_encoder = Motor1.getEncoder();
    //driveEncoder = Motor1.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
  }

  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
  }
  // --------------------------------------------------
  // ---------------------- AUTO ----------------------
  // --------------------------------------------------
  /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    
  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }
//Auto is working reliably, 
//TODO: Tune the values.
//TODO: Multi auto or add encoder. //see sample for a sendable chooser.
// It looks like the upper travel distance is ~16'
  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();
    System.out.println(time - startTime);
    if (time - startTime < 2.5) {
      //arm out (increase)
      Motor5.set(0.125);
    } else if (time - startTime > 2.5 && time - startTime < 4) {
      //arm in
      Motor5.set(-0.125);
    } else{
      Motor5.set(0);
    }
    
    if (time - startTime >= 2 && time - startTime < 2.5) {
      //cube out
      Motor6.set(0.5);
    } else {
      Motor6.set(0);
    }

    if (time - startTime > 5 && time - startTime < 7) {
      //drive back
      RobotDrive.arcadeDrive(0, -0.25);
    } else if (time - startTime >= 7 && time - startTime < 12){
      RobotDrive.arcadeDrive(0, -0.5);
    }else {
      RobotDrive.arcadeDrive(0, 0);
    }
  }


  // --------------------------------------------------
  // --------------------- TELEOP ---------------------
  // --------------------------------------------------

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    RobotDrive.arcadeDrive(-Controller1.getRightX()/2, -Controller1.getLeftY()/2);// drive code

    if (Controller1.getPOV() == 0) {
      //arm out
      Motor5.set(0.25);
    } else if (Controller1.getPOV() == 180) {
      //arm in
      Motor5.set(-0.25);
    } else {
      Motor5.set(0);
    }

    if (Controller1.getTriangleButton()) {
      //cube out
      Motor6.set(0.5);
    } else if (Controller1.getCircleButton()) {
      //cube in
      Motor6.set(-0.5);
    } else {
      Motor6.set(0);
    }
  }


  // --------------------------------------------------
  // ------------------- FUNCTIONS --------------------
  // --------------------------------------------------



  // --------------------------------------------------
}
