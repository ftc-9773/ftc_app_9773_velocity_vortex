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

    public enum BeaconAction{PUSH,RETRACT,IDLE}
    public class BeaconPushCmd{
        public BeaconAction beaconAction;
    }
    BeaconPushCmd beaconPushCmd = new BeaconPushCmd();


    public enum CapBallLiftAction{LIFT_LOCK,LIFT_UNLOCK,FORK_FOLD,FORK_AUTO_PLACEMENT,FORK_IDLE}
    public class CapBallLiftCmd{
        public CapBallLiftAction capBallLiftAction;
    }
    CapBallLiftCmd capBallLiftCmd = new CapBallLiftCmd();

    public enum HarvesterAction{INTAKE,OUTPUT,IDLE}
    public class HarvesterCmd{
        public HarvesterAction harvesterAction;
    }
    HarvesterCmd harvesterCmd = new HarvesterCmd();


    public class ParticleAccelCmd{
        public boolean activate;
    }
    ParticleAccelCmd particleAccelCmd = new ParticleAccelCmd();

    public enum ParticleReleaseStatus{RELEASE_WAIT_KEEP,KEEP}
    public class ParticleReleaseCmd{
        public ParticleReleaseStatus status;
    }
    ParticleReleaseCmd particleReleaseCmd = new ParticleReleaseCmd();

}
