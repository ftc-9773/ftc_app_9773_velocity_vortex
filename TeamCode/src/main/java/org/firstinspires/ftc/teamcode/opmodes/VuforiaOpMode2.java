package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.util.vision.BeaconState;
import org.firstinspires.ftc.teamcode.util.vision.VuforiaLocalizerImplSubclass;

import java.util.Arrays;
import java.util.List;

/**
 * Created by michaelzhou on 2/22/17.
 */

@Autonomous(name = "VuforiaOpMode2", group = "Autonomous")
public class VuforiaOpMode2 extends LinearOpMode{

//    private OpenGLMatrix lastKnownLocation;
//    private OpenGLMatrix phoneLocation;
    public static final Scalar blueLow = new Scalar(108,0,220);
    public static final Scalar blueHigh = new Scalar(178,255,255);

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup parameters to create localizer
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AbrtgIH/////AAAAAYXGG7tsoUbMqheoGcZNVXo2WXfiEHIwNSxiKDKqV4I8Z5iKsnnWyc9bu2wKiq2g2DICfNm/QLgt9SMJHGROCQgoA3bP38DUtYcOC4h5JwwoR6j0dTytoDJqNl4pR7EwO0BOBOmQPdqOejubaZWZpvNxUQnJBwtLKGGyMms+NPDgYZNCeSozTYuHYcWli+cl93GgfTwwm3j4yoxWMkUiQTa3yVQv7GewOn3KCnhaKsnZ84nkIlNkg1ees2K7u19zaiySu8Ielvi84AFAWS7rWXmDvcF95CbDKgeCJOd8V1tNQm5bwv+qcT8G1nA3gvmjeHVZMH5XXUR+A3YAD/5klU2OSmRBomj9FGYun5VjsAU+"; // Insert your own key here
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;//TODO: MAKE THIS FRONT FOR ROBOT TESTING/COMPETITION
        parameters.useExtendedTracking = false;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        VuforiaLocalizerImplSubclass vuforiaLocalizer = new VuforiaLocalizerImplSubclass(parameters);

        //Enable image detection
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforiaLocalizer.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        /*To access the image: you need to iterate through the images of the frame object:*/

        VuforiaLocalizer.CloseableFrame frame = vuforiaLocalizer.getFrameQueue().take(); //takes the frame at the head of the queue
        Image rgb = null;

        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }


        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        VuforiaTrackables visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        visionTargets.get(0).setName("Wheels");
        visionTargets.get(1).setName("Tools");
        visionTargets.get(2).setName("Lego");
        visionTargets.get(3).setName("Gears");


        waitForStart();

        visionTargets.activate();

        while(opModeIsActive()){
            // Setup listener and inform it of phone information
            for(VuforiaTrackable target : visionTargets){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) target.getListener()).getRawPose();

                if(pose!=null){
                    VectorF translation = pose.getTranslation();
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(target.getName() + "-Translation", translation);
                    telemetry.addData(target.getName() + "-Degrees", degreesToTurn);
                }
            }
            telemetry.update();
        }
    }

    public BeaconState analyzeBeacon(Image image, VuforiaTrackableDefaultListener listener, CameraCalibration cameraCalibration){
        OpenGLMatrix pose = listener.getRawPose();

        if(pose!=null && image!=null && image.getPixels()!=null){
            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float[][] corners = new float[4][2];
            corners[0] = Tool.projectPoint(cameraCalibration, rawPose, new Vec3F(-127, 276, 0)).getData();
            corners[1] = Tool.projectPoint(cameraCalibration, rawPose, new Vec3F(127, -276, 0)).getData();
            corners[2] = Tool.projectPoint(cameraCalibration, rawPose, new Vec3F(127, 276, 0)).getData();
            corners[3] = Tool.projectPoint(cameraCalibration, rawPose, new Vec3F(-127, -276, 0)).getData();

            Bitmap bm = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);//TODO: check if phone uses 565 or 888
            bm.copyPixelsFromBuffer(image.getPixels());

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm,crop);

            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            x = Math.max(x,0);
            y = Math.max(y,0);
            width = (x+width > crop.cols()) ? crop.cols()-x : width;
            height = (y+height > crop.rows()) ? crop.rows()-y : height;

            Mat cropped = new Mat(crop, new Rect((int)x, (int)y, (int)width, (int)height));

            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            Mat mask = new Mat();
            Core.inRange(cropped, blueLow, blueHigh, mask);//certain type of blue


        }
        return BeaconState.BEACON_NOTHING;
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w (pitch, roll, yaw)
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}
