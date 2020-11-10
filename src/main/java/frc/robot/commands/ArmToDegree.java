/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmToDegree extends CommandBase {
  private Arm m_arm;
  private double m_error;
  private double m_setPoint;
  private double errorSum;
  private double lastTimetamp;
  private double lastError;

  public ArmToDegree(Arm arm, double setPoint) {
    m_arm = arm;
    m_setPoint = setPoint;

    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    errorSum = 0;
    lastError = 0;
    lastTimetamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderPos = m_arm.encoder.get() * Constants.TICKS_TO_DEGREES;

    m_error = m_setPoint - encoderPos;

    double deltaTime = Timer.getFPGATimestamp() - lastTimetamp;

    if (Math.abs(m_error) < Constants.iLimit)
      errorSum += m_error * deltaTime;

    double errorRate = (m_error - lastError) / deltaTime;

    double outputSpeed = Constants.KP * m_error + Constants.KI * errorSum + Constants.KD * errorRate;

    m_arm.setMotorSpeed(outputSpeed);

    lastTimetamp = Timer.getFPGATimestamp();
    lastError = m_error;
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
