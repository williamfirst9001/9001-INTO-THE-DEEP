package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robotHardware;

public class Wrist extends SubsystemBase {


        private robotHardware robot = robotHardware.getInstance();
        public Wrist() {

        }

        public boolean isDone(){
            return true;
        }
        public void move(double pos){
            robot.wristServo.setPosition(pos);
        }

    }

