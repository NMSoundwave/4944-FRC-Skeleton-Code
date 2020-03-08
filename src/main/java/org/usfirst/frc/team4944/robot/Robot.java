package org.usfirst.frc.team4944.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4944.robot.custom.XboxController;
import org.usfirst.frc.team4944.robot.subsystems.DriveSystem;

public class Robot extends TimedRobot {

	// CONTROLLERS

	XboxController driver;
	XboxController operator;

	// SUBSYSTEMS

	OI oi;
	DriveSystem driveSystem;

	// Auto Commands
	Command autoCommand;

	@Override
	public void robotInit() {
		// CONTROLLERS INIT
		this.driver = new XboxController(0);
		this.operator = new XboxController(1);

		// SUBSYSTEMS INIT
		this.driveSystem = new DriveSystem();
		this.oi = new OI();

		// SmartDashboard
		//SmartDashboard.putNumber("Something", this.something);
		this.SmartDashboardDisplay();

	}

	@Override
	public void disabledInit() {
		this.SmartDashboardDisplay();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		this.SmartDashboardDisplay();
	}

	@Override
	public void autonomousInit() {
		Scheduler.getInstance().run();
		this.SmartDashboardDisplay();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		this.updateValues();
	}

	@Override
	public void teleopInit() {
		this.SmartDashboardDisplay();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		// Drive Code

		final double Y = -driver.getLeftStickY();
		final double X = driver.getRightStickX();
		this.driveSystem.setPower(X + Y, X - Y);

		// Update Values
		this.updateValues();
	}

	@Override
	public void testPeriodic() {
	}
		public void updateValues() {
		//this.something = SmartDashboard.getNumber("Something", this.something);

		// SMARTDASHBOARD

		// Displays all Smartdashboard Values

		this.SmartDashboardDisplay();

	}

	public void SmartDashboardDisplay() {

		// Something

		// SmartDashboard.putNumber("Something", this.something());
	}
}