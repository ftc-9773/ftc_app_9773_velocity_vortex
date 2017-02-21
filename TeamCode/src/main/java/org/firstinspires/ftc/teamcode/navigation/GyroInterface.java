package org.firstinspires.ftc.teamcode.navigation;

/**
 * Created by ftcrobocracy on 2/20/17.
 */

public interface GyroInterface {
    void initAfterStart();
    Navigation.GyroType getGyroType();
    double getYaw();
    double getPitch();
    boolean isGyroWorking();
    void testAndSetGyroStatus();
    double getUpdateCount();
    double getAngleTolerance();

    void setRobotOrientation(double targetAngle, double speed, NavigationChecks navigationChecks);

    void turnRobot(double angle, double speed, NavigationChecks navigationChecks);

    void goStraightPID(boolean driveBackwards, double degrees, float speed);
}
