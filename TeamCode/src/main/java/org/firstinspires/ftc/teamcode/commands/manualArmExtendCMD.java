package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class manualArmExtendCMD extends CommandBase {
    private elevator m_arm;
    public manualArmExtendCMD(elevator arm){
        m_arm = arm;
        addRequirements(m_arm);
    }
    @Override
    public void initialize(){

    }
}
