package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TwoMotorDrive {
    DcMotor motorL = null;
    DcMotor motorR = null;
    double frictionCoefficient;
    double maxSpeed;
    double minSpeed;

    public TwoMotorDrive(DcMotor motorL, DcMotor motorR, double maxSpeed, double minSpeed, double frictionCoefficient){
        this.motorL = motorL;
        this.motorR = motorR;
        this.frictionCoefficient = frictionCoefficient;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
    }

    public void drive(float speed, float direction){
        double left = (maxSpeed - minSpeed) * (speed + direction);
        double right = (maxSpeed - minSpeed) * (speed - direction);

        motorL.setPower(left);
        motorR.setPower(right);
    }

    public void driveToDistance(float speed, float direction, double distance){

    }
}
