package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class MineralDetector {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia = null;
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod = null;


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        try {
            vuforia = VuforiaFactory.initForTf(hardwareMap);

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                this.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
        } catch (Exception ex) {

        }
    }


    public void start() {
        if (tfod != null) {
            tfod.activate();
        }
    }


    /**
     * 获取金矿位置
     * 一次性侦测3个矿，则根据3个矿的位置进行判定
     *
     * @return No Left Center Right 之一
     */
    public String searchGoldMine() {
        String result = "No";

        if (tfod == null) {
            return result;
        }
        try {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions == null) {
                return result;
            }
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 3) {

                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Left");
                        result = "Left";
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Right");
                        result = "Right";
                    } else {
                        telemetry.addData("Gold Mineral Position", "Center");
                        result = "Center";
                    }
                }
            } else if (updatedRecognitions.size() == 2) {
                if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                    if (updatedRecognitions.get(0).getLeft() < updatedRecognitions.get(1).getLeft()) {
                        result = "Left";
                    } else {
                        result = "Center";
                    }

                } else if (updatedRecognitions.get(1).getLabel().equals(LABEL_GOLD_MINERAL)) {
                    if (updatedRecognitions.get(0).getLeft() < updatedRecognitions.get(1).getLeft()) {
                        result = "Center";
                    } else {
                        result = "Left";
                    }
                } else {
                    result = "Right";
                }

            }
            telemetry.update();


        } catch (Exception ex) {

        }

        return result;
    }

    /**
     * 获取金矿位置
     * 确保从左向右进行侦测的情况下，可以在屏幕中只有2个矿的情况下判定金矿位置
     *
     * @return No Left Center Right 之一
     */
    public String searchGoldMine2(boolean leftToRight) {
        String result = "No";
        if (tfod == null)
            return result;
        try {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions == null)
                return result;

            telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() != 2)
                return result;
            boolean is1gold = updatedRecognitions.get(0).getLabel().equals("LABEL_GOLD_MINERAL");
            boolean is2gold = updatedRecognitions.get(1).getLabel().equals("LABEL_GOLD_MINERAL");
            boolean isGoldFound =  is1gold ||is2gold;

            double pos1 = updatedRecognitions.get(0).getLeft();
            double pos2 = updatedRecognitions.get(1).getLeft();

            boolean isSilverLeft = !is1gold && pos1<pos2 || !is2gold && pos2<pos1;

            if (leftToRight) {
                if(!isGoldFound){
                    result = "Right";
                }else if(isSilverLeft){
                    result = "Center";
                }else{
                    result = "Left";
                }
            } else {
                if (!isGoldFound) {
                    result = "Left";
                } else if(isSilverLeft){
                    result = "Right";
                }else{
                    result = "Center";
                }
            }

            telemetry.update();

        } catch (Exception ex) {

        }

        return result;
    }

    public void stop() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }


    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        try {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        } catch (Exception ex) {

        }
    }
}
