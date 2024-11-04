package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class armCheckCMD extends CommandBase {
    private boolean done;
    public armCheckCMD(boolean done){
        this.done = done;
    }
    public void execute(){

    }
    public boolean isFinished(){
        return done;
    }
}
