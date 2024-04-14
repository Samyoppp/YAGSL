// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

    private final static CANSparkMax leftArmMotor = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(11, MotorType.kBrushless);

    public final static RelativeEncoder leftArmEncoder = leftArmMotor.getEncoder();
    private final SparkPIDController armController = leftArmMotor.getPIDController();

    double ArmkP = 0.1; // Proportional term
    double ArmkI = 0.0; // Integral term
    double ArmkD = 0.01; // Derivative term
    double ArmkIz = 0; // Integral zone
    double ArmkFF = 0.0; // Feed-forward
    double ArmkMaxOutput = .6;
    double ArmkMinOutput = -.5;

    public double armSetpoint = 0;

  public ArmSubsystem() {

    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    rightArmMotor.follow(leftArmMotor, true);

    leftArmMotor.setIdleMode(IdleMode.kCoast);
    rightArmMotor.setIdleMode(IdleMode.kCoast);

    leftArmMotor.setSmartCurrentLimit(40);
    rightArmMotor.setSmartCurrentLimit(40);

    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 52);
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);    

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();

    // PID configuration
    armController.setP(ArmkP);
    armController.setI(ArmkI);
    armController.setD(ArmkD);
    armController.setIZone(ArmkIz);
    armController.setFF(ArmkFF);
    armController.setOutputRange(ArmkMinOutput, ArmkMaxOutput);

    // Arm Encoder setup
    leftArmEncoder.setPosition(0);
    armController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void moveToTravel() {
    armController.setReference(-36, CANSparkMax.ControlType.kPosition);
  }

  public void moveToHome() {
    armController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
