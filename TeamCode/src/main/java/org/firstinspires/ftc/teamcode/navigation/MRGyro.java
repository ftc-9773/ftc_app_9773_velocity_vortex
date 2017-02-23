package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTCRobot;

/**
 * Created by Kids on 2/12/2017.
 */
public class MRGyro implements GyroInterface {
    private enum MRGyro_Status {STATUS_NOT_SET, WORKING, NOT_WORKING}

    ModernRoboticsI2cGyro gyro;
    LinearOpMode curOpMode;
    FTCRobot robot;
    Navigation navigation;
    public double straightPID_kp=0.005, turnPID_kp=0.005;
    double angleTolerance;
    private MRGyro_Status status=MRGyro_Status.STATUS_NOT_SET;
    private double updateCount;
    double prevYaw;
    ElapsedTime getYawTimer;
    // Do not bother to call getIntegratedZValue if it has been less than 20 milli seconds
    // since the last time it was called.
    final double GYRO_LATENCY = 20;

    public MRGyro(FTCRobot robot, LinearOpMode curOpMode, Navigation navigation,
                  double angleTolerance, double straightPID_kp, double turnPID_kp){
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.navigation = navigation;
        this.angleTolerance = angleTolerance;
        this.straightPID_kp = straightPID_kp;
        this.turnPID_kp = turnPID_kp;
        status=MRGyro_Status.STATUS_NOT_SET;
        updateCount = 0;


        gyro = (ModernRoboticsI2cGyro)curOpMode.hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (curOpMode.opModeIsActive() && gyro.isCalibrating())  {
            curOpMode.sleep(50);
            curOpMode.idle();
        }
        curOpMode.telemetry.addData("MRGyro: ", "Done with calibrating");
        gyro.resetZAxisIntegrator();
        DbgLog.msg("ftc9773: Done with initializing MR Gyro");
        DbgLog.msg("ftc9773: Current Yaw=%f, pitch=%f", (double)gyro.getHeading(), (double)gyro.rawX());
        curOpMode.telemetry.addData("MRGyro: ", "curYaw=%f, curPitch=%f", (double)gyro.getHeading(),
                (double) gyro.rawX());
        curOpMode.telemetry.update();

        getYawTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        prevYaw = gyro.getHeading();
        getYawTimer.reset();
    }

    // todo: find out how to return yaw and pitch angles
    // for Navx it is navx_device.getYaw() and navx_device.getPitch()

    @Override
    public void initAfterStart() {
        return;
    }

    @Override
    public Navigation.GyroType getGyroType() {
        return (Navigation.GyroType.MR_GYRO);
    }

    @Override
    public double getYaw() {
        double newYaw;
        newYaw = gyro.getHeading();
        if ((newYaw != prevYaw) || (getYawTimer.milliseconds() >= GYRO_LATENCY)) {
            prevYaw = newYaw;
            getYawTimer.reset();
            updateCount++;
        }
        else
            newYaw = prevYaw;

        return newYaw;
    }

    @Override
    public double getPitch() {
        // Normally pitch is obtained by rawY(), but due to the way the mr-gyro is mounted on
        // Robot V3, the rawX() value gives the pitch, not rawY().
//        return ((double)gyro.rawX());
        // ToDo: We need to figure out the way to get pitch value from MRGyro
        // Until then just return 0 pitch
        return (0);
    }

    @Override
    public boolean isGyroWorking() {
        return true;
    }

    @Override
    public double getUpdateCount() {
        // update count is valid only for navx-micro
        return (updateCount);
    }

    @Override
    public double getAngleTolerance() {
        return (angleTolerance);
    }

    @Override
    public void testAndSetGyroStatus() {
        if (gyro.isCalibrating()) {
            DbgLog.msg("ftc9773:  MR Gyro still calibrating; connection info=%s", gyro.getConnectionInfo());
            status = MRGyro_Status.NOT_WORKING;
        } else {
            DbgLog.msg("ft9773: MR Gyro is done with calibration");
            status = MRGyro_Status.WORKING;
        }
    }

    @Override
    public void setRobotOrientation(double targetYaw, double speed, NavigationChecks navigationChecks) {
        // The orientation is with respect to the initial autonomous starting position
        // The initial orientation of the robot at the beginning of the autonomous period
        // is '0'. targetAngle is between 0 to 360 degrees.
        double curYaw = getYaw();
        double yawDiff;
        double diff = targetYaw - curYaw;
        double angleToTurn = diff>180 ? diff-360 : diff<-180 ? diff+360 : diff;
//        turnRobot(angleToTurn, speed, navigationChecks);
        double rightPower, leftPower;
        if (angleToTurn > 0 && angleToTurn < 360) {
            // Spin clockwise
            leftPower = speed;
            rightPower = -1 * leftPower;
        }
        else if (angleToTurn < 0 && angleToTurn > -360) {
            // Spin counter clockwise
            rightPower = speed;
            leftPower = -1 * rightPower;
        } else {
            DbgLog.msg("ftc9773: angle %f is invalid!", angleToTurn);
            return;
        }
        DbgLog.msg("ftc9773: power left = %f, right = %f",leftPower, rightPower);
        robot.instrumentation.reset();
        while (!navigationChecks.stopNavigation()) {
            this.robot.driveSystem.turnOrSpin(leftPower, rightPower);
            robot.instrumentation.addInstrData();
            yawDiff = navigation.distanceBetweenAngles(getYaw(), targetYaw);
            if (yawDiff <= angleTolerance)
                break;
        }
        this.robot.driveSystem.stop();
        robot.instrumentation.printToConsole();
    }

    @Override
    public void turnRobot(double angle, double speed, NavigationChecks navigationChecks){
        double leftPower = 0.0, rightPower = 0.0;
        double startingYaw, targetYaw, yawDiff;
        double min_angleToTurn = 0.0;
//        LoopStatistics instr = new LoopStatistics();
        if (angle <0 && angle > -360){
            leftPower = speed;
            rightPower = -1*leftPower;
        } else if (angle > 0 && angle < 360){
            rightPower = speed;
            leftPower = -1*rightPower;
        } else if (angle == 0) {
            rightPower = 0;
            leftPower = 0;
        } else{
            DbgLog.msg("ftc9773: angle %f is invalid!", angle);
            return;
        }

        startingYaw = getYaw(); //not sure whether the range returned by getYaw is appropriate
        targetYaw = startingYaw + angle;
        if (targetYaw > 360){
            targetYaw %= 360;
        } else if (targetYaw < 0){
            while (targetYaw < 0){
                targetYaw += 360;
            }
        }
        DbgLog.msg("ftc9773: power left = %f, right = %f",leftPower, rightPower);
        DbgLog.msg("ftc9773: raw Yaw = %f, Starting yaw = %f, targetYaw = %f",
                gyro.rawZ(), startingYaw, targetYaw);

//        robot.repActions.startActions();
        while (curOpMode.opModeIsActive() && !navigationChecks.stopNavigation()) {
            this.robot.driveSystem.turnOrSpin(leftPower, rightPower);
//            robot.repActions.repeatActions();
            yawDiff = getYaw() - startingYaw; //warning
            if (yawDiff > min_angleToTurn)
                break;
        }

    }

    @Override
    public void goStraightPID(boolean driveBackwards, double degrees, float speed){
        double error = 0.0, correction = 0.0;
        double leftSpeed, rightSpeed;
        error = getYaw() - degrees;
        if (error > 180){
            error -= 360;
        } else if (error <-180){
            error += 360;
        }
        correction = this.straightPID_kp * error / 2;
        // Ensure that 0.25 <= speed <= 0.75 so that correction can be meanigful.
        speed = Range.clip(speed, 0.25f, 0.75f);
        leftSpeed = Range.clip(speed - correction, 0, 1);
        rightSpeed = Range.clip(speed + correction, 0, 1);

        if (!driveBackwards) {
            robot.driveSystem.turnOrSpin(leftSpeed, rightSpeed);
        } else {
            robot.driveSystem.turnOrSpin(-rightSpeed, -leftSpeed);
        }
    }
}