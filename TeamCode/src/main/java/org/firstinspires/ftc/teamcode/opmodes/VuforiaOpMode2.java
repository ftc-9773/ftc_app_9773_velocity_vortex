package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.List;

/**
 * Created by michaelzhou on 2/22/17.
 */

public class VuforiaOpMode2 extends LinearOpMode{
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    //    private VuforiaTrackable target;
//    private VuforiaTrackableDefaultListener listener;
    private List<VuforiaTrackableDefaultListener> listeners;
    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    @Override
    public void runOpMode() throws InterruptedException {
        setupVuforia();

        waitForStart();

        visionTargets.activate();

        while(opModeIsActive()){
            // Setup listener and inform it of phone information
            for(VuforiaTrackable target : visionTargets){
                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) target.getListener();
                OpenGLMatrix pose = listener.getPose();

                if(pose!=null){
                    VectorF translation = pose.getTranslation();
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(target.getName(), "-Translation", translation);
                    telemetry.addData(target.getName(), "-Degrees", degreesToTurn);
                }
            }
            telemetry.update();
        }
    }

    private void setupVuforia() throws InterruptedException {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AbrtgIH/////AAAAAYXGG7tsoUbMqheoGcZNVXo2WXfiEHIwNSxiKDKqV4I8Z5iKsnnWyc9bu2wKiq2g2DICfNm/QLgt9SMJHGROCQgoA3bP38DUtYcOC4h5JwwoR6j0dTytoDJqNl4pR7EwO0BOBOmQPdqOejubaZWZpvNxUQnJBwtLKGGyMms+NPDgYZNCeSozTYuHYcWli+cl93GgfTwwm3j4yoxWMkUiQTa3yVQv7GewOn3KCnhaKsnZ84nkIlNkg1ees2K7u19zaiySu8Ielvi84AFAWS7rWXmDvcF95CbDKgeCJOd8V1tNQm5bwv+qcT8G1nA3gvmjeHVZMH5XXUR+A3YAD/5klU2OSmRBomj9FGYun5VjsAU+"; // Insert your own key here
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;//TODO: MAKE THIS FRONT FOR ROBOT TESTING/COMPETITION
        parameters.useExtendedTracking = false;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

//        //Enable image detection
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
//        vuforiaLocalizer.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
//
//        /*To access the image: you need to iterate through the images of the frame object:*/
//
//        VuforiaLocalizer.CloseableFrame frame = vuforiaLocalizer.getFrameQueue().take(); //takes the frame at the head of the queue
//        Image rgb = null;
//
//        long numImages = frame.getNumImages();
//
//
//        for (int i = 0; i < numImages; i++) {
//            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                rgb = frame.getImage(i);
//                break;
//            }//if
//        }//for


        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        visionTargets.get(0).setName("Wheels");
        visionTargets.get(1).setName("Tools");
        visionTargets.get(2).setName("Lego");
        visionTargets.get(3).setName("Gears");


    }
}
