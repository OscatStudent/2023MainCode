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
  private float driveSpeed;
  private float rotationSpeed;
  //private float turnDifference = 10;
  private int rampSpeedAdj;

  // -------------------------------------------------
  // --------------------- ROBOT ---------------------
  // -------------------------------------------------
  private RelativeEncoder m1_encoder;
  private RelativeEncoder m2_Encoder;
  private final double kDriveTick2Feet = 1.0 / 42 * 4 * 6 * Math.PI / 12;

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
    m1_encoder = Motor1.getEncoder();
    m2_Encoder = Motor2.getEncoder();

    //driveEncoder = Motor1.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
  }

  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Encoder Position: ", m1_encoder.getPosition());
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
    Motor5.setIdleMode(IdleMode.kBrake);
    m1_encoder.setPosition(0);
    m2_Encoder.setPosition(0);

  }
//Auto is working reliably, 
//TODO: Tune the values.
//TODO: Multi auto or add encoder. //see sample for a sendable chooser.
// It looks like the upper travel distance is ~16'
  @Override
  public void autonomousPeriodic() {
    //encoder is 42 counts per rev
    double leftPosition = m1_encoder.getPosition() * kDriveTick2Feet;
    double rightPosition = m2_Encoder.getPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2;

    if (distance < 5){
      RobotDrive.arcadeDrive(0, 0.25);
    }





    /*************WORKING one piece auto (6-3 points)***********/
    /*double time = Timer.getFPGATimestamp();
    System.out.println(time - startTime);
    if (time - startTime < 2) {
      //arm out (increase)
      Motor5.set(0.25);
    } else if (time - startTime > 2.5 && time - startTime < 4.5) {
      //arm in
      Motor5.set(-0.25);
    } else{
      Motor5.set(0);
    }
    
    if (time - startTime >= 2.1 && time - startTime < 2.5) {
      //cube out
      Motor6.set(1);
    } else {
      Motor6.set(0);
    }

    if (time - startTime > 2.5 && time - startTime < 3.5) {
      //drive back
      RobotDrive.arcadeDrive(0, -0.25);
    } else if (time - startTime >= 3.5 && time - startTime < 11){
      RobotDrive.arcadeDrive(0, -0.4);
    }else {
      RobotDrive.arcadeDrive(0, 0);
    }
    /***************************************/



  }


  // --------------------------------------------------
  // --------------------- TELEOP ---------------------
  // --------------------------------------------------

  @Override
  public void teleopInit() {
    Motor5.setIdleMode(IdleMode.kBrake);
  }
  
  @Override
  public  void disabledInit() {
    Motor5.setIdleMode(IdleMode.kCoast);
  }
  
  @Override
  public void teleopPeriodic() {
    /*******WORKING drive code (non-ramp)*********/
    
    /*if (Controller1.getR1Button()) {
      RobotDrive.arcadeDrive(-Controller1.getRightX(), -Controller1.getLeftY());// drive code, full speed
      SmartDashboard.putString("pressed", "pressed :)");
    } else{
      RobotDrive.arcadeDrive((-Controller1.getRightX()*3)/4, -(Controller1.getLeftY()*3)/4);// drive code, lower speed
      SmartDashboard.putString("unpressed", "unpressed :)");
    }*/
    /**********************************/
    
    /************new code****************/
    
    driveSpeed += ((-Controller1.getLeftY() - driveSpeed) / (20 - rampSpeedAdj));
    rotationSpeed += ((-Controller1.getRightX() - rotationSpeed) / 10);
    
    RobotDrive.arcadeDrive(rotationSpeed, driveSpeed);

    /*************************************/

   

    if (Controller1.getR1Button()) {
      //arm out
      Motor5.set(0.25);
    } else if (Controller1.getR2Button()) {
      //arm in
      Motor5.set(-0.25);
    } else {
      Motor5.set(0);
    }

    if (Controller1.getL1Button()) {
      //cube out
      Motor6.set(0.5);
    } else if (Controller1.getL2Button()) {
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
