package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class highChamberCMD extends CommandBase {
    private elevator m_arm;
    private Wrist m_wrist;

    public highChamberCMD(elevator arm, Wrist wrist) {
        m_arm = arm;
        m_wrist = wrist;
        addRequirements(m_arm,m_wrist);
    }

    @Override
    public void initialize() {
        m_arm.setPivotGains(constants.pivotConstants.P, constants.pivotConstants.I, constants.pivotConstants.D);
        m_wrist.move(constants.wristPoints.basket);
    }
    @Override
    public void execute(){
        m_wrist.move(constants.wristPoints.basket);
        m_arm.goToSetpoint(constants.elevatorSetpoints.armSetpoints.highChamber,constants.elevatorSetpoints.pivotSetpoints.chamber);

    }
    @Override
    public boolean isFinished(){
        return m_arm.isDone();
    }

}
