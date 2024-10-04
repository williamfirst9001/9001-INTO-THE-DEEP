package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class stowCMD extends CommandBase {
    private elevator m_arm;

    public stowCMD(elevator arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setElevatorGains(constants.armConstants.middle.P, constants.armConstants.middle.I, constants.armConstants.middle.D);
    }

    @Override
    public void execute() {
        m_arm.goToSetpoint(0, 0);
    }


}
