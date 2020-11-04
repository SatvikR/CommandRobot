/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmToDegree extends CommandBase {
  private Arm m_arm;
  private double m_error;
  private double m_setPoint;

  public ArmToDegree(Arm arm, double setPoint) {
    m_arm = arm;
    m_setPoint = setPoint;

    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderPos = m_arm.encoder.get() * Constants.TICKS_TO_DEGREES;

    m_error = m_setPoint - encoderPos;

    double outputSpeed = Constants.KP * m_error;

    m_arm.setMotorSpeed(outputSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
