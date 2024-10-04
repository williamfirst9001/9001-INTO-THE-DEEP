package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class highChamberCMD extends CommandBase {
    private elevator m_arm;

    public highChamberCMD(elevator arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setElevatorGains(constants.armConstants.chambers.P,constants.armConstants.chambers.I,constants.armConstants.chambers.D);
        m_arm.setPivotGains(constants.armConstants.chambers.P,constants.armConstants.chambers.I,constants.armConstants.chambers.D);
    }
    @Override
    public void execute(){
        m_arm.goToSetpoint(constants.elevatorSetpoints.armSetpoints.highChamber,constants.elevatorSetpoints.pivotSetpoints.chamber);
    }
    @Override
    public boolean isFinished(){
        return m_arm.isDone();
    }
}
