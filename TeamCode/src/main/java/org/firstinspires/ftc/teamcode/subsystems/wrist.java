package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.robotHardware;

public class wrist extends SubsystemBase {


        private robotHardware robot = robotHardware.getInstance();
        public wrist() {

        }

        public boolean isDone(){
            return true;
        }
        public void move(double pos){
            robot.wristServo.setPosition(pos);
        }
        public double getPos(){
            return robot.claw.getPosition();
        }
    }

