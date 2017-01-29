package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.attachments.CapBallLift;
import org.firstinspires.ftc.teamcode.attachments.ParticleRelease;

/**
 * Created by michaelzhou on 1/29/17.
 */

public class DriveCommand {

    public class DriveSysCommand{
        public double position, yaw;
    }
    DriveSysCommand driveSysCommand = new DriveSysCommand();

    public enum BeaconAction{PUSH,RETRACT,NONE}
    public class BeaconPushCmd{
        public BeaconAction beaconAction;
    }
    BeaconPushCmd beaconPushCmd = new BeaconPushCmd();


    public class CapBallLiftCmd{
        public double position;
    }
    CapBallLiftCmd capBallLiftCmd = new CapBallLiftCmd();


    public class HarvesterCmd{
        //0 - None, 1 - up, -1 - down
        public int status;
    }
    HarvesterCmd harvesterCmd = new HarvesterCmd();

    public class ParticleAccelCmd{
        public boolean activate;
    }
    ParticleAccelCmd particleAccelCmd = new ParticleAccelCmd();

    public enum ParticleReleaseStatus{RELEASE,KEEP}
    public class ParticleReleaseCmd{
        public ParticleReleaseStatus status;
    }
    ParticleReleaseCmd particleReleaseCmd = new ParticleReleaseCmd();

    public enum EndGameStatus{NONE,RUN,STOP}
    public class RunEndGame{
        public EndGameStatus endGameStatus;
    }
    RunEndGame runEndGame = new RunEndGame();
}
