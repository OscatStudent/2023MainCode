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
  //private CANSparkMax Motor4 = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax Motor5 = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax Motor6 = new CANSparkMax(6, MotorType.kBrushed);

  private MotorControllerGroup Left = new MotorControllerGroup(Motor2);
  private MotorControllerGroup Right = new MotorControllerGroup(Motor1);
  DifferentialDrive RobotDrive = new DifferentialDrive(Left, Right);
 
  private double startTime;
  private float driveSpeed;
  private float rotationSpeed;
  private double driveDifference;
  private double rotationDifference;
  //private float turnDifference = 10;
  private int rampSpeedAdj;

  // -------------------------------------------------
  // --------------------- ROBOT ---------------------
  // -------------------------------------------------
  private RelativeEncoder m1_Encoder;
  private RelativeEncoder m2_Encoder;
  //private final double kDriveTick2Feet = (1.0 / 42) * 4 * (6 * Math.PI) / 12;
  private final double kDriveTick2Feet = (1 / 42) * 4 * (6 * Math.PI) * (1 / 12);
  
//test
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("kPDefault", .1);
    
    Motor1.restoreFactoryDefaults();
    Motor2.restoreFactoryDefaults();
    //Motor3.restoreFactoryDefaults();
    //Motor4.restoreFactoryDefaults();

    Motor2.setInverted(true);
    Motor2.burnFlash();

    Motor6.setIdleMode(IdleMode.kBrake);

    CameraServer.startAutomaticCapture();
        /**
    * In order to read encoder values an encoder object is created using the 
    * getEncoder() method from an existing CANSparkMax object
    */
    m1_Encoder = Motor1.getEncoder();
    m2_Encoder = Motor2.getEncoder();
    //m2_Encoder.setInverted(true);


    //driveEncoder = Motor1.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.getNumber("kPGet", 0);

    //SmartDashboard.putNumber("Encoder 1 Position: ", m1_Encoder.getPosition());
    //SmartDashboard.putNumber("Encoder 2 Position: ", m2_Encoder.getPosition());
  
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
    
    Motor1.setIdleMode(IdleMode.kBrake);
    Motor2.setIdleMode(IdleMode.kBrake);
    Motor5.setIdleMode(IdleMode.kBrake);
    
    m1_Encoder.setPosition(0);
    m2_Encoder.setPosition(0);
    
    errorSum = 0;
    lastError = 0;
    lastTimeStamp = Timer.getFPGATimestamp();


  }
//Auto is working reliably, 
//TODO: Tune the values.
//TODO: Multi auto. //see sample for a sendable chooser.
// It looks like the upper travel distance is ~16'

  //final double kP = 0.2;//0.5
  final double kI = 0.05;//0.5
  final double kD = 0.01;//0.1
  final double iLimit = 1;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimeStamp = 0;
  double lastError = 0;
  double kP;

  @Override
  public void autonomousPeriodic() {

    //TODO: be sure to adjust before comp
    //encoder is 42 counts per rev
    double leftPosition = (m1_Encoder.getPosition() / 42) * 4 * (6 * Math.PI) / 12;//kDriveTick2Feet;
    double rightPosition = (m2_Encoder.getPosition() / 42) * 4 * (6 * Math.PI) / 12;//kDriveTick2Feet;
    //double leftPosition = m1_Encoder.getPosition() * (1 / 42) * 4 * (6 * Math.PI) * (1 / 12);//kDriveTick2Feet;
    //double rightPosition = m2_Encoder.getPosition() * kDriveTick2Feet;
    //kDriveTick2Feet = (1 / 42) * 4 * (6 * Math.PI) * (1 / 12);
    
    double distance = (leftPosition + rightPosition) / 2;
    SmartDashboard.putNumber("Distance: ", distance);

    /*if (distance < 5){
      RobotDrive.arcadeDrive(0.25, 0);
    }else {
      RobotDrive.arcadeDrive(0, 0);
    }*/
    //*****************************/

    /*if (Controller1.getR1Button()) {
      setpoint = 10;
    } else if (Controller1.getR2Button()) {
      setpoint = 0;
    }*/
    setpoint = 5;
    SmartDashboard.putNumber("Setpoint: ", setpoint);

    // calculations
    double error = setpoint - distance;
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;

    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double errorRate = (error - lastError) / dt;
    SmartDashboard.putNumber("kP: " , kP);
    double outputSpeed = kP * error /*test: + kI * errorSum;*/ /*+ kD * errorRate;*/;
    SmartDashboard.putNumber("Output Speed:  ", outputSpeed);

    //output to motors
    //RobotDrive.arcadeDrive(outputSpeed, 0);
    RobotDrive.arcadeDrive(0, 0);

    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = error;

    /***********************************************************/
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
    Motor1.setIdleMode(IdleMode.kBrake);
    Motor2.setIdleMode(IdleMode.kBrake);
    Motor5.setIdleMode(IdleMode.kBrake);
    m1_Encoder.setPosition(0);
    m2_Encoder.setPosition(0);

  }
  
  @Override
  public  void disabledInit() {
    Motor5.setIdleMode(IdleMode.kCoast);
    Motor1.setIdleMode(IdleMode.kCoast);
    Motor2.setIdleMode(IdleMode.kCoast);
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
    //
    /************new code****************/
    driveDifference = (Controller1.getLeftY() - driveSpeed);
    rotationDifference = (Controller1.getRightX() - rotationSpeed);

    if(driveDifference > 0.5){ // was .1
      rampSpeedAdj = 0;
    }else {
      rampSpeedAdj = 19;
    }

    driveSpeed += (driveDifference / (20 - rampSpeedAdj));
    rotationSpeed += (rotationDifference / 10);
    


    //driveSpeed += ((-Controller1.getLeftY() - driveSpeed) / (20 - rampSpeedAdj));
    //rotationSpeed += ((-Controller1.getRightX() - rotationSpeed) / 10);
    
    RobotDrive.arcadeDrive(rotationSpeed, driveSpeed/2);

    double leftPosition = m1_Encoder.getPosition() * kDriveTick2Feet;
    double rightPosition = m2_Encoder.getPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2;
    SmartDashboard.putNumber("Distance: ", distance);

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
