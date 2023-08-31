// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
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
  private CANSparkMax Motor3 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax Motor4 = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax Motor5 = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax Motor6 = new CANSparkMax(6, MotorType.kBrushed);

  private MotorControllerGroup Left = new MotorControllerGroup(Motor2);
  private MotorControllerGroup Right = new MotorControllerGroup(Motor1);

  DifferentialDrive RobotDrive = new DifferentialDrive(Left, Right);

  // -------------------------------------------------
  // --------------------- ROBOT ---------------------
  // -------------------------------------------------

  @Override
  public void robotInit() {
    Motor1.restoreFactoryDefaults();
    Motor2.restoreFactoryDefaults();
    Motor3.restoreFactoryDefaults();
    Motor4.restoreFactoryDefaults();

    Motor6.setIdleMode(IdleMode.kBrake);

    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {

  }


  // --------------------------------------------------
  // ---------------------- AUTO ----------------------
  // --------------------------------------------------

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    /*int sval = 0;
    if (sval < 30) {
      Motor1.set(0.05);
      Motor4.set(-0.05);
      Motor3.set(0.05);
      Motor2.set(-0.05);
      sval += 1;
    }*/
  }


  // --------------------------------------------------
  // --------------------- TELEOP ---------------------
  // --------------------------------------------------

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    RobotDrive.arcadeDrive(-Controller1.getRightX()/2, Controller1.getLeftY()/2);

    if (Controller1.getPOV() == 0) {
      Motor5.set(0.125);
    } else if (Controller1.getPOV() == 180) {
      Motor5.set(-0.125);
    } else {
      Motor5.set(0);
    }

    if (Controller1.getTriangleButton()) {
      Motor6.set(0.5);
    } else if (Controller1.getCircleButton()) {
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
