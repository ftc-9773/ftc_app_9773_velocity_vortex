package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.attachments.CapBallLift;
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

    public void getDSCmd(){
//        Gamepad gamepad1 = curOpMode.gamepad1;
//        Gamepad gamepad2 = curOpMode.gamepad2;

//        if(curOpMode.gamepad1.back) robot.stateMachine.transition(curOpMode);

        //save the commands
        getBeaconCmd();
        getCapBallLiftCmd();
        getHarvesterCmd();
        getParticleAccelCmd();
        getParticleReleaseCmd();
    }

    public DriveCommand.BeaconPushCmd getBeaconCmd(){
        command.beaconPushCmd.beaconAction = curOpMode.gamepad2.x ? DriveCommand.BeaconAction.PUSH :
                curOpMode.gamepad2.b ? DriveCommand.BeaconAction.RETRACT :
                        DriveCommand.BeaconAction.IDLE;
        return command.beaconPushCmd;
    }

    public DriveCommand.CapBallLiftCmd getCapBallLiftCmd(){
        command.capBallLiftCmd.capBallLiftAction = curOpMode.gamepad2.right_bumper ? DriveCommand.CapBallLiftAction.LIFT_LOCK :
                curOpMode.gamepad2.left_bumper ? DriveCommand.CapBallLiftAction.LIFT_UNLOCK:
                        curOpMode.gamepad2.y ? DriveCommand.CapBallLiftAction.FORK_FOLD :
                                curOpMode.gamepad2.a ? DriveCommand.CapBallLiftAction.FORK_AUTO_PLACEMENT :
                                        DriveCommand.CapBallLiftAction.FORK_IDLE;
        return command.capBallLiftCmd;
    }

    public DriveCommand.HarvesterCmd getHarvesterCmd(){
        command.harvesterCmd.harvesterAction = curOpMode.gamepad2.dpad_down ? DriveCommand.HarvesterAction.INTAKE :
                curOpMode.gamepad2.dpad_up ? DriveCommand.HarvesterAction.OUTPUT :
                        DriveCommand.HarvesterAction.IDLE;
        return command.harvesterCmd;
    }

    public DriveCommand.ParticleAccelCmd getParticleAccelCmd(){
        if(curOpMode.gamepad1.dpad_up)
            command.particleAccelCmd.activate = true;
        if(curOpMode.gamepad1.dpad_down)
            command.particleAccelCmd.activate =  false;
        return command.particleAccelCmd;
    }

    public DriveCommand.ParticleReleaseCmd getParticleReleaseCmd(){
        if(curOpMode.gamepad1.a)
            command.particleReleaseCmd.status = DriveCommand.ParticleReleaseStatus.RELEASE_WAIT_KEEP;
        if(curOpMode.gamepad1.y)
            command.particleReleaseCmd.status = DriveCommand.ParticleReleaseStatus.KEEP;
        return command.particleReleaseCmd;
    }
}
