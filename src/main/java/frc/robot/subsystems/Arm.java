/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*if you are reading this, hi*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private WPI_TalonSRX mainArm;
    private DigitalInput limitSwitch;
    private Counter counter;

    public Arm() {
        limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);
        mainArm = new WPI_TalonSRX(Constants.ARM_MOTOR_PORT);
        counter = new Counter(limitSwitch);

    }

    public boolean isSwitchSet() {
        return counter.get() > 0;

    }

    public void initializeCounter() {
        counter.reset();
    }

    public void armUp() {
        mainArm.set(0.5);
    }

    public void armDown() {
        mainArm.set(-0.5);
    }

    public void armStop() {
        mainArm.set(0.0);
    }

    protected void initDefaultCommand() {
    }

}
