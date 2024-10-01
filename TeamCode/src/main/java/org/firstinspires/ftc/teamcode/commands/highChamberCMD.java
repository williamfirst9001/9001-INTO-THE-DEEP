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
        m_arm.setElevatorGains(constants.armConstants.chambers.kV,constants.armConstants.chambers.kA,constants.armConstants.chambers.kStatic);
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
