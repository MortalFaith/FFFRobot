package org.firstinspires.ftc.teamcode;

import android.telecom.TelecomManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TLog {

    static Telemetry telemetry = null;
    static boolean isDebug = false;

    public static void init(Telemetry tele){
        telemetry = tele;
    }

    public static void log(String capation,String message){
        if(telemetry==null)
            return;
        log(capation,message,false);
    }

    public static void log(String capation,String message, boolean isUpdate){
        if(telemetry==null)
            return;
        if (!isDebug){
            return;
        }
        telemetry.addData(capation,message);

        if(isUpdate){
            telemetry.update();
        }
    }

    public static void update() {
        if(telemetry==null)
            return;
        telemetry.update();
    }
}
