package org.firstinspires.ftc.teamcode.util;

import android.hardware.Camera;
import android.os.Build;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 * Created by pranavb on 10/26/16.
 */
@SuppressWarnings("deprecation")
public class CameraRead {
    Camera cameraDevice = null;
    Camera.PictureCallback jpegCallback = null;

    public CameraRead(){
        cameraDevice = Camera.open();
    }

    public void takePicture(final String filePath, int cameraId) throws IOException{
        if (Build.VERSION.SDK_INT <= 21){
            jpegCallback = new Camera.PictureCallback() {
                @Override
                public void onPictureTaken(byte[] bytes, Camera camera) {
                    FileOutputStream fileOutputStream = null;
                    try {
                        fileOutputStream = new FileOutputStream(String.format(filePath + "beacon.jpeg"));

                        fileOutputStream.write(bytes);
                        fileOutputStream.close();
                    } catch (Exception e){
                        e.printStackTrace();
                    }
                }
            };
        }
        cameraDevice.takePicture(null, null, jpegCallback);
    }
}
