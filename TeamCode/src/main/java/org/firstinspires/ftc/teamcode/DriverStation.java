package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.StateMachine;

/**
 * Created by michaelzhou on 1/29/17.
 */

public class DriverStation {
    public DriveCommand command = new DriveCommand();
    FTCRobot robot;
    LinearOpMode curOpMode;


    public DriverStation(FTCRobot robot, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
    }

    public void switchState(){
        if(curOpMode.gamepad1.back)
            robot.stateMachine.transition(curOpMode);
    }

    public void getAndApplyDSCmd(){
        Gamepad gamepad1 = curOpMode.gamepad1;
        Gamepad gamepad2 = curOpMode.gamepad2;
        if(gamepad1.back) robot.stateMachine.transition(curOpMode);

    }

    private DriveCommand getBeaconCmd(){
        command.beaconPushCmd.beaconAction = DriveCommand.BeaconAction.PUSH;
        return command;
    }
}
