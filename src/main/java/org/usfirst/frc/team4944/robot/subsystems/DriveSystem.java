package org.usfirst.frc.team4944.robot.subsystems;

import org.usfirst.team4944.robot.PID.BasicPID;
import org.usfirst.team4944.robot.PID.DrivePID;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveSystem extends Subsystem {

	// MOTORS
	TalonFX leftMotor1;
	TalonFX leftMotor2;
	TalonFX rightMotor1;
	TalonFX rightMotor2;
	// GYRO
	AHRS gyro;
	// DRIVE PIDS
	DrivePID leftPID;
	DrivePID rightPID;
	// GYRO PID
	BasicPID anglePID;
	// CONSTANTS
	final double maxPow = 0.90;
	final double wheelDiam = 6;
	final double wheelCircum = Math.PI*this.wheelDiam;
	final double ticksPerRotation = 2048;
	final double speedThreshold = 1;
	// VALUES
	double currentLeft = 0;
	double currentRight = 0;
	double currentAngle = 0;
	double lastLeft = 0;
	double lastRight = 0;
	double lastAngle = 0;

	public DriveSystem() {
		// MOTORS
		// Comp Bot
		this.leftMotor1 = new TalonFX(3);
		this.leftMotor2 = new TalonFX(4);
		this.rightMotor1 = new TalonFX(1);
		this.rightMotor2 = new TalonFX(2);
		// ENCODERS
		this.leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		this.rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		// GYRO
		this.gyro = new AHRS(Port.kMXP);
		// ANGLE PID
		this.anglePID = new BasicPID(1 / 500, 1 / 500, 1 / 500);
	}

	public void setLeftPower(double power) {
		this.leftMotor1.set(ControlMode.PercentOutput, power);
		this.leftMotor2.set(ControlMode.PercentOutput, power);
	}

	public void setRightPower(double power) {
		rightMotor1.set(ControlMode.PercentOutput, power);
		rightMotor2.set(ControlMode.PercentOutput, power);
	}

	public void setPower(double lPower, double rPower) {
		setLeftPower(lPower * this.maxPow);
		setRightPower(rPower * this.maxPow);
	}

	public double getAngle() {
		return gyro.getAngle();
	}

	public int getLeftEncoder() {
		return leftMotor1.getSelectedSensorPosition();
	}

	public int getRightEncoder() {
		return rightMotor1.getSelectedSensorPosition();
	}

	public void updateValues(){
		this.lastLeft = this.currentLeft;
		this.lastRight = this.currentRight;
		this.lastAngle = this.currentAngle;
		this.currentLeft = this.getLeftEncoder();
		this.currentRight = this.getRightEncoder();
	}

	public double getLeftSpeed() {
		return this.lastLeft - this.currentLeft;
	}

	public double getRightSpeed() {
		return this.lastRight - this.currentRight;
	}

	public boolean getDoneDriveing(){
		if(Math.abs(this.getRightSpeed()) < this.speedThreshold && Math.abs(this.getLeftSpeed()) < this.speedThreshold){
			return true;
		}else{
			return false;
		}
	}

	public void stop() {
		setPower(0, 0);
	}

	public double convertInchesToEncoderCount(double inches){
		double ticksPerInch = (this.ticksPerRotation/this.wheelCircum);
		return inches*ticksPerInch;
	}

	@Override
	protected void initDefaultCommand() {
	}
}