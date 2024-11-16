package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.commands.driveCMD;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.robotHardware;

import java.util.List;

public class Wrist extends SubsystemBase {


        private robotHardware robot = robotHardware.getInstance();
        private double startTime;
        private double setPoint = 0;
        public Wrist() {

        }
        public void update(){
            robot.wristServo.setPosition(setPoint);
        }
        public void setSetPoint(double point){
            setPoint = point;
        }
    public void setSetPoint(List<Double> vals){
        setPoint = (vals.get(2));
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
        public void move(double pos){
            robot.wristServo.setPosition(pos);
        }

        public void move(List<Double> vals){
            robot.wristServo.setPosition(vals.get(2));
        }


    }

