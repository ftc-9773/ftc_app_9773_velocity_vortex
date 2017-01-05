package org.firstinspires.ftc.teamcode.util.vision;

import android.graphics.Bitmap;
import android.provider.Settings;
import android.widget.ArrayAdapter;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
//import org.firstinspires.ftc.robotcontroller.internal.vision.*;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

/**
 * Created by pb8xe_000 on 12/30/2016.
 */

/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class Vision {
//    FrameGrabber frameGrabber=null;
    LinearOpMode curOpMode=null;
    FTCRobot robot=null;
    public String VUFORIA_LICENSE_KEY= "AbrtgIH/////AAAAAYXGG7tsoUbMqheoGcZNVXo2WXfiEHIwNSxiKDKqV4I8Z5iKsnnWyc9bu2wKiq2g2DICfNm/QLgt9SMJHGROCQgoA3bP38DUtYcOC4h5JwwoR6j0dTytoDJqNl4pR7EwO0BOBOmQPdqOejubaZWZpvNxUQnJBwtLKGGyMms+NPDgYZNCeSozTYuHYcWli+cl93GgfTwwm3j4yoxWMkUiQTa3yVQv7GewOn3KCnhaKsnZ84nkIlNkg1ees2K7u19zaiySu8Ielvi84AFAWS7rWXmDvcF95CbDKgeCJOd8V1tNQm5bwv+qcT8G1nA3gvmjeHVZMH5XXUR+A3YAD/5klU2OSmRBomj9FGYun5VjsAU+";
    VuforiaLocalizer.Parameters params = null;
    VuforiaTrackables beacons = null;
    VuforiaLocalizerImplSubclass vuforia = null;

    public Vision(LinearOpMode curOpMode, FTCRobot robot){
        this.curOpMode = curOpMode;
        this.robot = robot;
//        this.frameGrabber= FtcRobotControllerActivity.frameGrabber;
        params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = new VuforiaLocalizerImplSubclass(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
    }

    /*public BeaconColorResult.BeaconColor[] getVisionBeaconColors(){
        frameGrabber.grabSingleFrame();
        while (!frameGrabber.isResultReady() && curOpMode.opModeIsActive()){
            curOpMode.sleep(5);
        }

        org.firstinspires.ftc.robotcontroller.internal.vision.ImageProcessorResult
                imageProcessorResult = frameGrabber.getResult();
        BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();

        BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
        BeaconColorResult.BeaconColor rightColor = result.getRightColor();

        BeaconColorResult.BeaconColor[] results = {leftColor, rightColor};

        curOpMode.telemetry.addData("Beacon color result: ", results.toString());
        curOpMode.telemetry.update();


        return results;
    }*/

    public BeaconColorResult.BeaconColor[][] getBothBeaconColors(){
        BeaconColorResult.BeaconColor[][] beaconColors = new BeaconColorResult.BeaconColor[4][2];
        beacons.activate();
        curOpMode.sleep(1000);
        long frameTime = System.currentTimeMillis();
        Bitmap bitmap = null;
        if (vuforia.rgb != null){
            bitmap = Bitmap.createBitmap(vuforia.rgb.getWidth(), vuforia.rgb.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(vuforia.rgb.getPixels());
        }
        for(int i=0;i<beacons.size();i++){
            VuforiaTrackable beacon = beacons.get(i);
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getRawPose();

            BeaconColorResult result;
            Mat original = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);
            Utils.bitmapToMat(bitmap, original);

            if (pose != null){
                DbgLog.msg("ftc9773: pose != null");
                Matrix34F rawPose = new Matrix34F();
                float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                rawPose.setData(poseData);

                float[][] corners = new float[4][2];

                corners[0] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,276,0)).getData(); //upper left
                corners[1] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,276,0)).getData(); //upper right
                corners[2] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,92,0)).getData(); //lower right
                corners[3] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,92,0)).getData(); //lower left

                float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
                float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
                float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
                float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

                x = Math.max(x, 0);
                y = Math.max(y, 0);
                width = (x + width > original.cols())? original.cols() - x : width;
                height = (y + height > original.rows())? original.rows() - y : height;

                Mat cropped = new Mat(original, new Rect((int) x, (int) y, (int) width, (int) height));

                BeaconProcessor imageProcessor = new BeaconProcessor();

                result = imageProcessor.process(frameTime, cropped, true).getResult();

                Mat frame = imageProcessor.process(frameTime, cropped, false).getFrame();

                Mat tmp2 = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);

                Core.transpose(frame, original);
                Imgproc.resize(original, tmp2, tmp2.size(), 0, 0, 0);
                Core.transpose(tmp2, frame);
            }
            else{
                BeaconProcessor imageProcessor = new BeaconProcessor();
                result = imageProcessor.process(frameTime,original, true).getResult();
            }
            BeaconColorResult beaconColorResult = result;
            beaconColors[i][0] = beaconColorResult.getLeftColor();
            beaconColors[i][1] = beaconColorResult.getRightColor();
        }
        beacons.deactivate();
        return beaconColors;
    }
}
