package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class armMoveCMD extends CommandBase {
    private elevator m_arm;
    private double armPoint,pivotPoint;

    public armMoveCMD(elevator arm,double armP,double pivotP) {
        m_arm = arm;
        addRequirements(m_arm);
        armPoint = armP;
        pivotPoint = pivotP;
    }

    @Override
    public void initialize() {
        m_arm.setPivotGains(constants.pivotConstants.P,constants.pivotConstants.I,constants.pivotConstants.D);

    }
    @Override
    public void execute(){
        m_arm.goToSetpoint(armPoint,pivotPoint);

    }

}
