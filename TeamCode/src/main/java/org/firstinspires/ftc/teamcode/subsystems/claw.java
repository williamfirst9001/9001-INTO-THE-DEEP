package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;

public class claw extends SubsystemBase {





        private robotHardware robot = robotHardware.getInstance();
        public claw() {

        }

        public boolean isDone(){
            return true;
        }
        public void update(){
            if(globals.clawState == globals.ClawState.OPEN){
                robot.claw.setPosition(constants.clawPoints.openPos);
            } else {
                robot.claw.setPosition(constants.clawPoints.closePos);
            }
        }
    }

