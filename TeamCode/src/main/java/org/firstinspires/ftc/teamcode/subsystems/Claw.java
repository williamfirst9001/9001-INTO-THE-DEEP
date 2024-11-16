package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.robotHardware;

public class Claw extends SubsystemBase {





        private robotHardware robot = robotHardware.getInstance();
        private double startTime;
        public Claw() {

        }

        public boolean isDone(){
            return true;
        }
        public void setStartTime(double val){
            startTime = val;
        }
        public double getStartTime(){
            return startTime;
        }
        public void close(){
            robot.claw.setPosition(constants.clawPoints.closePos);
        }
        public void open(){
            robot.claw.setPosition(constants.clawPoints.openPos);
        }
        public double getPos(){
            return robot.claw.getPosition();
        }
    }

