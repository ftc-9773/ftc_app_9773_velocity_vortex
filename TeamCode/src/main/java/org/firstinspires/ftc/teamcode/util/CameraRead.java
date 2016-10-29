package org.firstinspires.ftc.teamcode.util;

import android.hardware.Camera;
import android.os.Build;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;

/**
 * Created by pranavb on 10/26/16.
 */
@SuppressWarnings("deprecation")
public class CameraRead {
    Camera cameraDevice = null;
    Camera.PictureCallback jpegCallback = null;

    public CameraRead(final String filePath){
        if (Build.VERSION.SDK_INT <= 21){
            cameraDevice = Camera.open();
            jpegCallback = new Camera.PictureCallback() {
                @Override
                public void onPictureTaken(byte[] bytes, Camera camera) {
                    FileOutputStream fileOutputStream = null;
                    try {
                        fileOutputStream = new FileOutputStream(String.format(filePath + "beacon.jpeg"));
                    } catch (Exception e){
                        e.printStackTrace();
                    }
                }
            };
        }
    }

    public void takePicture(int cameraId){
        //cameraDevice.takePicture();
    }
}
