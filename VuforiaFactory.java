package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.*;

public class VuforiaFactory {


    private static final String VUFORIA_KEY = "AegblNT/////AAABmQbC4gavNk/2oo4dxCeTz2iGYV+lkOtnpdmw2WqsQ5CjQ0t6bRbHhpWDKJdkCBBg3B1Lt/JXq+Na0bEv0VAFZwwnG3QK0RAmXEnHlSzjRxeb2c7PGaUQfwXIerK6R8Wc9HQfkR4Pgrw/9Ux0/QemHNU0nogkiIoxY2aLsYqsFBtUUnXVq6QxQ4MQ5ITOXI4dz+r/A1304CIrDAVEqemBGsu21xBdVJWFWuct9v1LQ5EAbHyX8nnf6uF2ybqHL8sJayYcfbdk5ZaDDUx/O/kFLJHTF7wTkAHLCnMvOndHFqoLpc7SUD0zliqGP/sI5t1hlxHl9ntQQDTlefoRE+eItIogKXNbTUMawanzTbKZYzSd";

    private static VuforiaLocalizer localizer = null;

    /**
     */
    public static VuforiaLocalizer getInstance(HardwareMap hardwareMap) {

        if(localizer !=null){
            return localizer;
        }
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        try {
            Parameters parameters = new Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            //parameters.cameraDirection   = CameraDirection.BACK;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
            localizer = ClassFactory.getInstance().createVuforia(parameters);
        } catch (Exception ex) {

        }
        return localizer;

    }


    /**
     */
    public static VuforiaLocalizer init(HardwareMap hardwareMap) {

        localizer = null;
        if (Vuforia.isInitialized()) {
            Vuforia.deinit();
        }
        Vuforia.init();
        return localizer;

    }


    /**
     */
    public static VuforiaLocalizer initForTf(HardwareMap hardwareMap) {

        VuforiaLocalizer localizer = null;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        try {
            Parameters parameters = new Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            //parameters.cameraDirection   = CameraDirection.BACK;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
            localizer = ClassFactory.getInstance().createVuforia(parameters);
        } catch (Exception ex) {

        }
        return localizer;

    }


    /**
     * Initialize the Vuforia localization engine.
     */
    public static VuforiaLocalizer initForLocate(HardwareMap hardwareMap) {
        if (Vuforia.isInitialized()) {
            Vuforia.deinit();
        }
        VuforiaLocalizer localizer = null;
        try {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            Parameters parameters = new Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            //parameters.cameraDirection   = CameraDirection.BACK;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

            //  Instantiate the Vuforia engine
            localizer = ClassFactory.getInstance().createVuforia(parameters);
        } catch (Exception ex) {

        }
        return localizer;

    }


}
