package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;

import java.util.ArrayList;
import java.util.List;

public class NavigationChecks {
    public enum NavChecksSupported {CHECK_OPMODE_INACTIVE, CHECK_ROBOT_TILTING, CHECK_TIMEOUT,
        CHECK_WHITElINE, CHECK_DEGREES_TURNED, CHECK_DISTANCE_TRAVELLED,
        CROSSCHECK_NAVX_WITH_ENCODERS, CHECK_DRIVESYS_BUSY}
    LinearOpMode curOpMode;
    FTCRobot robot;
    Navigation navigationObj;
    public List<NavCheckBaseClass> criteriaToCheck = new ArrayList<NavCheckBaseClass>();
    public NavCheckBaseClass stopNavCriterion;

    public class NavCheckBaseClass {
        public NavChecksSupported navcheck;
        public boolean stopNavigation() { return (false);}
        public void reset() { return; }
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
        public void reset() {
            this.timer.reset();
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

    public class CheckDrivesysBusy extends NavCheckBaseClass {
        @Override
        public boolean stopNavigation() {
            if (!robot.driveSystem.isBusy()) {
                return (true);
            } else {
                return (false);
            }
        }

        @Override
        public void reset() { return;}
    }

    public class CheckNavxWhileTurning extends NavCheckBaseClass {
        double degreesToCheck;
        DriveSystem.ElapsedEncoderCounts elapsedCounts;
        double navxYaw;
        NavxMicro navxMicro;
        public CheckNavxWhileTurning(double degreesToCheck) {
            this.degreesToCheck = degreesToCheck;
            elapsedCounts = robot.driveSystem.getNewElapsedCountsObj();
            elapsedCounts.reset();
            navxMicro = navigationObj.navxMicro;
            navxYaw = navxMicro.getModifiedYaw();
            navcheck = NavChecksSupported.CROSSCHECK_NAVX_WITH_ENCODERS;
        }

        @Override
        public void reset() {
            elapsedCounts.reset();
            navxYaw = navxMicro.getModifiedYaw();
        }

        @Override
        public boolean stopNavigation() {
            double encoder_degreesTurned = Math.abs(elapsedCounts.getDegreesTurned());
            double navx_degreesTurned = navigationObj.distanceBetweenAngles(navxYaw,
                    navxMicro.getModifiedYaw());
            double diff = Math.abs(encoder_degreesTurned - navx_degreesTurned);
            if (diff > Math.abs(degreesToCheck)) {
                DbgLog.msg("ftc9773: encoder degrees: %f, navx degrees: %f", encoder_degreesTurned, navx_degreesTurned);
                return (true);
            } else {
                return (false);
            }
        }
    }

    public class CheckRobotTilting extends NavCheckBaseClass {
        double pitchDegrees;
        NavxMicro navxMicro;
        double navxPitch;
        // TODO: 12/29/16 Investigate the feasibility of using phone's builtin sensors to detect tilting

        public CheckRobotTilting(double pitchDegrees) {
            this.pitchDegrees = pitchDegrees;
            navxMicro = navigationObj.navxMicro;
            navxPitch = navxMicro.getPitch();
            navcheck = NavChecksSupported.CHECK_ROBOT_TILTING;
        }

        @Override
        public void reset() {
            return;
        }

        @Override
        public boolean stopNavigation() {
            if (Math.abs(navxMicro.getPitch() - navxPitch) > pitchDegrees) {
                return (true);
            } else {
                return (false);
            }
        }
    }

    public class OpmodeInactiveCheck extends NavCheckBaseClass {
        @Override
        public boolean stopNavigation() {
            if (curOpMode.opModeIsActive()) {
                return (false);
            } else {
                return (true);
            }
        }

        @Override
        public void reset() {
            return;
        }

    }

    /**
     * Check if the required distance in inches has been travelled
     */
    public class EncoderCheckForDistance extends NavCheckBaseClass {
        double distanceInInches;
        DriveSystem.ElapsedEncoderCounts elapsedCounts;

        public EncoderCheckForDistance(double distanceInInches) {
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
                DbgLog.msg("ftc9773: distanceTravelled = %f", distanceTravelled);
                elapsedCounts.printCurrentEncoderCounts();
                return (true);
            } else {
                return (false);
            }
        }

        @Override
        public void reset() {
            elapsedCounts.reset();
        }
    }

    public class CheckForWhiteLine extends NavCheckBaseClass {
        @Override
        public boolean stopNavigation() {
            if (navigationObj.lf.onWhiteLine()) {
                return (true);
            } else {
                return (false);
            }
        }

        @Override
        public void reset() {
            return;
        }
    }

    public NavigationChecks(FTCRobot robot, LinearOpMode curOpMode, Navigation navigationObj) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.navigationObj = navigationObj;
        stopNavCriterion = null;
    }

    public boolean stopNavigation() {
        for (NavCheckBaseClass e: this.criteriaToCheck) {
            if (e.stopNavigation()) {
                this.stopNavCriterion = e;
                return (true);
            }
        }
        return (false);
    }
    public void reset() {
        for (NavCheckBaseClass e: this.criteriaToCheck) {
            e.reset();
        }
    }

    public void addNewCheck(NavigationChecks.NavCheckBaseClass navCheck) {
        this.criteriaToCheck.add(navCheck);
    }

    public void removeCheck(NavCheckBaseClass navCheck) {
        this.criteriaToCheck.remove(navCheck);
    }
}
