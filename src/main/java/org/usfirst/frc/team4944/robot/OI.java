package org.usfirst.frc.team4944.robot;

import org.usfirst.frc.team4944.robot.custom.ToggleCommand;
import org.usfirst.frc.team4944.robot.custom.TriggerCommand;
import org.usfirst.frc.team4944.robot.custom.XboxController;

import edu.wpi.first.wpilibj.command.Command;


public class OI {
    XboxController driver, operator;
    boolean prevAButton, prevBButton, prevXButton, prevYButton, prevRBButton, prevLBButton, prevRTButton, prevLTButton;
    Command aCommandOn, aCommandOff, bCommandOn, bCommandOff, xCommandOn, xCommandOff, yCommandOn, yCommandOff;
    Command lbCommandOn, lbCommandOff, rbCommandOn, rbCommandOff, rtCommandOn, rtCommandOff, ltCommandOn, ltCommandOff;
    Command rmCommandOn, rmCommandOff, lmCommandOn, lmCommandOff;
    ToggleCommand toggleA, toggleB, toggleX, toggleY, toggleRB, toggleLB, toggleRT, toggleLT, toggleLM, toggleRM;
    TriggerCommand leftTrigger, rightTrigger;

    public OI() {

        // Controllers

        this.driver = new XboxController(0);
        this.operator = new XboxController(1);

        // Button Toggles

        this.prevAButton = false;
        this.prevBButton = false;
        this.prevXButton = false;
        this.prevYButton = false;
        this.prevRBButton = false;
        this.prevLBButton = false;
        this.prevLTButton = false;
        this.prevRTButton = false;

        // Commands

        // A -- TEXT HERE (Driver)
        // this.aCommandOn = null;
        // this.aCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToA(this.aCommandOn);
        // this.driver.addWhenReleasedToA(this.aCommandOff);
        // Toggle
        // this.toggleA = new ToggleCommand(this.aCommandOn, this.aCommandOff);
        // this.driver.addCommandToA(this.toggleA);

        // B -- TEXT HERE (Driver)
        // this.bCommandOn = null;
        // this.bCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToB(this.bCommandOn);
        // this.driver.addWhenReleasedToB(this.bCommandOff);
        // Toggle
        // this.toggleB = new ToggleCommand(this.bCommandOn, this.bCommandOff);
        // this.driver.addCommandToB(this.toggleB);

        // X -- TEXT HERE (Driver)
        // this.xCommandOn = null;
        // this.xCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToX(this.xCommandOn);
        // this.driver.addWhenReleasedToX(this.xCommandOff);
        // Toggle
        // this.toggleX = new ToggleCommand(this.xCommandOn, this.xCommandOff);
        // this.driver.addCommandToX(this.toggleX);

        // Y -- TEXT HERE (Driver)
        // this.yCommandOn = null;
        // this.yCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToY(this.yCommandOn);
        // this.driver.addWhenReleasedToY(this.yCommandOff);
        // Toggle
        // this.toggleY = new ToggleCommand(this.yCommandOn, this.yCommandOff);
        // this.driver.addCommandToY(this.toggleY);

        // RB -- TEXT HERE (Driver)
        // this.rbCommandOn = null;
        // this.rbCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToRightBumper(this.rbCommandOn);
        // this.driver.addWhenReleasedToRightBumper(this.rbCommandOff);
        // Toggle
        // this.toggleRB = new ToggleCommand(this.rbCommandOn, this.rbCommandOff);
        // this.driver.addCommandToRightBumper(this.toggleRB);

        // LB -- TEXT HERE (Driver)
        // this.lbCommandOn = null;
        // this.lbCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToLeftBumper(this.lbCommandOn);
        // this.driver.addWhenReleasedToLeftBumper(this.lbCommandOff);
        // Toggle
        // this.toggleLB = new ToggleCommand(this.lbCommandOn, this.lbCommandOff);
        // this.driver.addCommandToLeftBumper(this.toggleLB);

        // RT -- TEXT HERE (Driver)
        // this.rtCommandOn = null;
        // this.rtCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToRightTrigger(this.rtCommandOn);
        // this.driver.addWhenReleasedToRightTrigger(this.rtCommandOff);
        // Toggle
        // this.toggleRT = new ToggleCommand(this.rtCommandOn, this.rtCommandOff);
        // this.driver.addCommandToRightTrigger(this.toggleRT);

        // LT -- TEXT HERE (Driver)
        // this.ltCommandOn = null;
        // this.ltCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToLeftTrigger(this.ltCommandOn);
        // this.driver.addWhenReleasedToLeftTrigger(this.ltCommandOff);
        // Toggle
        // this.toggleLT = new ToggleCommand(this.ltCommandOn, this.ltCommandOff);
        // this.driver.addCommandToLeftTrigger(this.toggleLT);

        // RM -- TEXT HERE (Driver)
        // this.rmCommandOn = null;
        // this.rmCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToRightMenu(this.rmCommandOn);
        // this.driver.addWhenReleasedToRightMenu(this.rmCommandOff);
        // Toggle
        // this.toggleRM = new ToggleCommand(this.rmCommandOn, this.rmCommandOff);
        // this.driver.addCommandToRightMenu(this.toggleRM);

        // LM -- TEXT HERE (Driver)
        // this.lmCommandOn = null;
        // this.lmCommandOff = null;
        // Tap
        // this.driver.addWhenHeldToLeftMenu(this.lmCommandOn);
        // this.driver.addWhenReleasedToLeftMenu(this.lmCommandOff);
        // Toggle
        // this.toggleLM = new ToggleCommand(this.lmCommandOn, this.lmCommandOff);
        // this.driver.addCommandToLeftMenu(this.toggleLM);

        // A -- TEXT HERE (Operator)
        // this.aCommandOn = null;
        // this.aCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToA(this.aCommandOn);
        // this.operator.addWhenReleasedToA(this.aCommandOff);
        // Toggle
        // this.toggleA = new ToggleCommand(this.aCommandOn, this.aCommandOff);
        // this.operator.addCommandToA(this.toggleA);

        // B -- TEXT HERE (Operator)
        // this.bCommandOn = null;
        // this.bCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToB(this.bCommandOn);
        // this.operator.addWhenReleasedToB(this.bCommandOff);
        // Toggle
        // this.toggleB = new ToggleCommand(this.bCommandOn, this.bCommandOff);
        // this.operator.addCommandToB(this.toggleB);

        // X -- TEXT HERE (Operator)
        // this.xCommandOn = null;
        // this.xCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToX(this.xCommandOn);
        // this.operator.addWhenReleasedToX(this.xCommandOff);
        // Toggle
        // this.toggleX = new ToggleCommand(this.xCommandOn, this.xCommandOff);
        // this.operator.addCommandToX(this.toggleX);

        // Y -- TEXT HERE (Operator)
        // this.yCommandOn = null;
        // this.yCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToY(this.yCommandOn);
        // this.operator.addWhenReleasedToY(this.yCommandOff);
        // Toggle
        // this.toggleY = new ToggleCommand(this.yCommandOn, this.yCommandOff);
        // this.operator.addCommandToY(this.toggleY);

        // RB -- TEXT HERE (Operator)
        // this.rbCommandOn = null;
        // this.rbCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToRightBumper(this.rbCommandOn);
        // this.operator.addWhenReleasedToRightBumper(this.rbCommandOff);
        // Toggle
        // this.toggleRB = new ToggleCommand(this.rbCommandOn, this.rbCommandOff);
        // this.operator.addCommandToRightBumper(this.toggleRB);

        // LB -- TEXT HERE (Operator)
        // this.lbCommandOn = null;
        // this.lbCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToLeftBumper(this.lbCommandOn);
        // this.operator.addWhenReleasedToLeftBumper(this.lbCommandOff);
        // Toggle
        // this.toggleLB = new ToggleCommand(this.lbCommandOn, this.lbCommandOff);
        // this.operator.addCommandToLeftBumper(this.toggleLB);

        // RT -- TEXT HERE (Operator)
        // this.rtCommandOn = null;
        // this.rtCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToRightTrigger(this.rtCommandOn);
        // this.operator.addWhenReleasedToRightTrigger(this.rtCommandOff);
        // Toggle
        // this.toggleRT = new ToggleCommand(this.rtCommandOn, this.rtCommandOff);
        // this.operator.addCommandToRightTrigger(this.toggleRT);

        // LT -- TEXT HERE (Operator)
        // this.ltCommandOn = null;
        // this.ltCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToLeftTrigger(this.ltCommandOn);
        // this.operator.addWhenReleasedToLeftTrigger(this.ltCommandOff);
        // Toggle
        // this.toggleLT = new ToggleCommand(this.ltCommandOn, this.ltCommandOff);
        // this.operator.addCommandToLeftTrigger(this.toggleLT);

        // RM -- TEXT HERE (Operator)
        // this.rmCommandOn = null;
        // this.rmCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToRightMenu(this.rmCommandOn);
        // this.operator.addWhenReleasedToRightMenu(this.rmCommandOff);
        // Toggle
        // this.toggleRM = new ToggleCommand(this.rmCommandOn, this.rmCommandOff);
        // this.operator.addCommandToRightMenu(this.toggleRM);

        // LM -- TEXT HERE (Operator)
        // this.lmCommandOn = null;
        // this.lmCommandOff = null;
        // Tap
        // this.operator.addWhenHeldToLeftMenu(this.lmCommandOn);
        // this.operator.addWhenReleasedToLeftMenu(this.lmCommandOff);
        // Toggle
        // this.toggleLM = new ToggleCommand(this.lmCommandOn, this.lmCommandOff);
        // this.operator.addCommandToLeftMenu(this.toggleLM);
    }
}
