package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class scoreChamberCMD extends CommandBase {
    private NanoClock clock = NanoClock.system();


        private elevator m_arm;
        private Claw m_claw;

        public scoreChamberCMD(elevator arm, Claw claw) {
            m_arm = arm;
            m_claw = claw;
            addRequirements(m_arm,m_claw);
        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
            m_arm.goToSetpoint(0);
            if(m_arm.armDone())
                m_arm.goToSetpoint(0, 100);
        }


    }


