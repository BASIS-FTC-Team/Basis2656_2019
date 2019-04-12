package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class GoldDetector{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Ac4SO4P/////AAAAmYo+Dd1E4komrpVteq5yhwyKezLLi2tGgobkZ33Cw+FfGBDlxL282Ow6UJycv6OKKGKtALv6scAq+4cHivE+XPOu6008QHCI0P6yx8X9vb8IKrLWM7dC2ZaWp1Em6rVZFS9q/SnAWVjU1J2oZFNKK5t2jsfpcFV+vN+ZCyNXT+kBsk8mLKwesanwvrCoja1i4Ycs/8FJt7G7EVL2H+wQtGH1Q2sy/AGhJRXAiOyZHM97UBhOptoY9trn6omnmlO3/z8Gr+ntJEqXA/GdyHbJkRcI3bG+vxU3fhUsX3W5Gm7dUs3dX2po7Kz1Q38ABtrLuwpJd1abPHZvSt1vrKe8p5JJtk9ABMZcgPXBBL7eOUr6";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private int idDetect;

    HardwareMap hwMap = null;

    int init(HardwareMap Map, Config config) {
        hwMap = Map;

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            // telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            idDetect = -1;
        }

        if (tfod != null) {
            tfod.activate();
            idDetect = 0;
        }
        return idDetect;
    }

    void doDetect(){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
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
                                //telemetry.addData("Gold Mineral Position", "Left");
                                idDetect = 1;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                //telemetry.addData("Gold Mineral Position", "Right");
                                idDetect = 2;
                            } else {
                                //telemetry.addData("Gold Mineral Position", "Center");
                                idDetect = 3;
                            }
                        }
                    }
                   // telemetry.update();
                }
            }
        }

    public int getId(){
        return  idDetect;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}

