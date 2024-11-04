package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class stowCMD extends CommandBase {
    private elevator m_arm;
    private Wrist m_wrist;

    public stowCMD(elevator arm,Wrist wrist) {
        m_arm = arm;
        m_wrist = wrist;
        addRequirements(m_arm,m_wrist);
    }

    @Override
    public void initialize() {
        m_wrist.move(constants.wristPoints.stow);
    }

    @Override
    public void execute() {
        m_wrist.move(constants.wristPoints.stow);
        m_arm.setSetPoint(0,0);
    }


}
