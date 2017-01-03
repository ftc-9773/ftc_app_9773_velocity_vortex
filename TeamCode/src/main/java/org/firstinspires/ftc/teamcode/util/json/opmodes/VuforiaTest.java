package org.firstinspires.ftc.teamcode.util.json.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.util.vision.VuforiaLocalizerImplSubclass;

/**
 * Created by pranavburugula on 1/2/2017.
 */

/*
 * Copyright (c) 2017 Robocracy 9773
 */
@Autonomous(name = "Vuforia Test", group = "Autonomous")
public class VuforiaTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AbrtgIH/////AAAAAYXGG7tsoUbMqheoGcZNVXo2WXfiEHIwNSxiKDKqV4I8Z5iKsnnWyc9bu2wKiq2g2DICfNm/QLgt9SMJHGROCQgoA3bP38DUtYcOC4h5JwwoR6j0dTytoDJqNl4pR7EwO0BOBOmQPdqOejubaZWZpvNxUQnJBwtLKGGyMms+NPDgYZNCeSozTYuHYcWli+cl93GgfTwwm3j4yoxWMkUiQTa3yVQv7GewOn3KCnhaKsnZ84nkIlNkg1ees2K7u19zaiySu8Ielvi84AFAWS7rWXmDvcF95CbDKgeCJOd8V1tNQm5bwv+qcT8G1nA3gvmjeHVZMH5XXUR+A3YAD/5klU2OSmRBomj9FGYun5VjsAU+";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()){
            for (VuforiaTrackable beacon : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();

                if (pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beacon.getName() + "-Translation", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beacon.getName() + "-Degrees", degreesToTurn);
                }
            }
            telemetry.update();
        }
    }
}
