package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;

public class NavigationChecks {
    public enum NavChecksSupported {CHECK_OPMODE_INACTIVE, CHECK_ROBOT_TILTING, CHECK_TIMEOUT,
        CHECK_WHITElINE, CHECK_DEGREES_TURNED, CHECK_DISTANCE_TRAVELLED, CROSSCHECK_NAVX_WITH_ENCODERS}
    LinearOpMode curOpMode;
    FTCRobot robot;
    Navigation navigationObj;
    long timeoutMilliSec;
    NavCheckBaseClass[] criteriaToCheck;
    NavCheckBaseClass StopNavCriterion;

    public class NavCheckBaseClass {
        public NavChecksSupported navcheck;
        public boolean stopNavigation() { return (false);}
    }

    public class TimeoutCheck extends NavCheckBaseClass {
        private ElapsedTime timer;
        private long timeoutMillis;

        public TimeoutCheck(long timeoutMillis) {
            this.timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            this.timer.reset();
            this.timeoutMillis = timeoutMillis;
            navcheck = NavChecksSupported.CHECK_TIMEOUT;
        }

        @Override
        public boolean stopNavigation() {
            if (timer.milliseconds() >= timeoutMillis) {
                return (true);
            } else {
                return (false);
            }
        }
    }

    public class checkNavxWhileTurning extends NavCheckBaseClass {
        double degreesToCheck;
        DriveSystem.ElapsedEncoderCounts elapsedCounts;
        double navxYaw;
        NavxMicro navxMicro;
        public checkNavxWhileTurning(double degreesToCheck) {
            this.degreesToCheck = degreesToCheck;
            elapsedCounts = robot.driveSystem.getNewElapsedCountsObj();
            elapsedCounts.reset();
            navxMicro = navigationObj.navxMicro;
            navxYaw = navxMicro.getModifiedYaw();
            navcheck = NavChecksSupported.CROSSCHECK_NAVX_WITH_ENCODERS;
        }

        @Override
        public boolean stopNavigation() {
            double encoder_degreesTurned = Math.abs(elapsedCounts.getDegreesTurned());
            double navx_degreesTurned = navigationObj.distanceBetweenAngles(navxYaw,
                    navxMicro.getModifiedYaw());
            double diff = Math.abs(encoder_degreesTurned - navx_degreesTurned);
            if (diff > Math.abs(degreesToCheck)) {
                return (true);
            } else {
                return (false);
            }
        }
    }

    public class encoderCheckForDistance extends NavCheckBaseClass {
        double distanceInInches;
        DriveSystem.ElapsedEncoderCounts elapsedCounts;

        public encoderCheckForDistance(double distanceInInches) {
            this.distanceInInches = distanceInInches;
            elapsedCounts = robot.driveSystem.getNewElapsedCountsObj();
            elapsedCounts.reset();
            navcheck = NavChecksSupported.CHECK_DISTANCE_TRAVELLED;
        }

        @Override
        public boolean stopNavigation() {
            double distanceTravelled = elapsedCounts.getDistanceTravelledInInches();
            // Check for both magnitude and sign
            if ((Math.abs(distanceTravelled) >= Math.abs(distanceInInches)) &&
                    ((distanceTravelled / distanceInInches) > 0)) {
                return (true);
            } else {
                return (false);
            }
        }
    }

    public NavigationChecks(FTCRobot robot, LinearOpMode curOpMode, Navigation navigationObj,
                            long timeoutMilliSec, NavCheckBaseClass[] criteriaToCheck) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.navigationObj = navigationObj;
        this.timeoutMilliSec = timeoutMilliSec;
        this.criteriaToCheck = criteriaToCheck;
        StopNavCriterion = null;
    }

    public boolean checkExceptions() {
        for (NavCheckBaseClass e: this.criteriaToCheck) {
            if (e.stopNavigation()) {
                this.StopNavCriterion = e;
                return (true);
            }
        }
        return (false);
    }
}
