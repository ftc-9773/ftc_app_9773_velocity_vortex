package org.firstinspires.ftc.teamcode.opmodes;

/**
 * Created by michaelzhou on 2/19/17.
 */
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.ImageTarget;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

/*
 * This OpMode was written for the VuforiaDemo Basics video. This demonstrates basic principles of
 * using VuforiaDemo in FTC.
 */
@Autonomous(name = "VuforiaOpMode", group = "Autonomous")
public class VuforiaOpMode extends LinearOpMode
{
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
//    private VuforiaTrackable target;
//    private VuforiaTrackableDefaultListener listener;
    private List<VuforiaTrackableDefaultListener> listeners;
    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private static final String VUFORIA_KEY = "AbrtgIH/////AAAAAYXGG7tsoUbMqheoGcZNVXo2WXfiEHIwNSxiKDKqV4I8Z5iKsnnWyc9bu2wKiq2g2DICfNm/QLgt9SMJHGROCQgoA3bP38DUtYcOC4h5JwwoR6j0dTytoDJqNl4pR7EwO0BOBOmQPdqOejubaZWZpvNxUQnJBwtLKGGyMms+NPDgYZNCeSozTYuHYcWli+cl93GgfTwwm3j4yoxWMkUiQTa3yVQv7GewOn3KCnhaKsnZ84nkIlNkg1ees2K7u19zaiySu8Ielvi84AFAWS7rWXmDvcF95CbDKgeCJOd8V1tNQm5bwv+qcT8G1nA3gvmjeHVZMH5XXUR+A3YAD/5klU2OSmRBomj9FGYun5VjsAU+"; // Insert your own key here

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;

    public void runOpMode() throws InterruptedException
    {
        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        while(opModeIsActive())
        {
            OpenGLMatrix latestLocation = null;
            for(VuforiaTrackable target : visionTargets){
                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) target.getListener();
                // Ask the listener for the latest information on where the robot is
                latestLocation = listener.getUpdatedRobotLocation();

                // The listener will sometimes return null, so we check for that to prevent errors
                if(latestLocation != null){
                    lastKnownLocation = latestLocation;
                    float[] coordinates = lastKnownLocation.getTranslation().getData();
                    robotX = coordinates[0];
                    robotY = coordinates[1];
                    robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

                    // Send information about whether the target is visible, and where the robot is
                    telemetry.addData("Tracking " + target.getName(), listener.isVisible());
                    telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
                    telemetry.addData("Distance from target: ", "%f", getDistance(lastKnownLocation, target.getLocation()));
                    DbgLog.msg("ftc9773: Location: ", formatMatrix(lastKnownLocation));
                }
            }

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }
    }

    private void setupVuforia()
    {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        visionTargets.get(0).setName("Wheels"); // 0 corresponds to the wheels target
        visionTargets.get(0).setLocation(createMatrix(0, 500, 0, 90, 0, 90));
        visionTargets.get(1).setName("Tools");
        visionTargets.get(1).setLocation(createMatrix(0, -500, 0, -90, 0, 90));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        listeners = new ArrayList<>();
        // Setup listener and inform it of phone information
        for(VuforiaTrackable target : visionTargets){
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) target.getListener();
            listeners.add(listener);
            listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }
    }

    private float getDistance(OpenGLMatrix obj, OpenGLMatrix target){
        float[] myData = obj.getData();
        float[] targetData = target.getData();
        float x = Math.abs(targetData[0] - myData[0]);
        float y = Math.abs(targetData[1] - myData[1]);
        float z = Math.abs(targetData[2] - myData[2]);
        return (float) Math.sqrt(x*x + z*z);
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
