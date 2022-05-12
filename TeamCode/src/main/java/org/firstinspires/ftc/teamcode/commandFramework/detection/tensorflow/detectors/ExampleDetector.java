package org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.detectors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.detectors.Detector;
import org.firstinspires.ftc.teamcode.commandFramework.detection.tensorflow.classification.Classifier;

import java.util.ArrayList;
import java.util.List;

public class ExampleDetector implements Runnable{
    Telemetry telemetry;
    private Detector tfDetector = null;
    private HardwareMap hardwareMap;

    private boolean isRunning = true;

    private String modelFileName = "model_unquant.tflite";
    private String labelFileName = "labels.txt";
    private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
    private static final String LABEL_ZAYN = "Zayn";
    private static final String LABEL_AHMED = "Ahmed";

    private String result = LABEL_ZAYN; //just a default value.

    private LinearOpMode caller = null;

    public ExampleDetector(HardwareMap hMap, LinearOpMode caller, Telemetry t) throws Exception {
        hardwareMap = hMap;
        telemetry = t;
        initDetector();
        activateDetector();
        this.caller = caller;
    }

    public ExampleDetector(HardwareMap hMap, LinearOpMode caller, Telemetry t, String model, String labels) throws Exception {
        hardwareMap = hMap;
        telemetry = t;
        setModelFileName(model);
        setLabelFileName(labels);
        initDetector();
        activateDetector();
        this.caller = caller;
    }

    public void facialDetectionThread() {

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (isRunning && caller.opModeIsActive()) {
            if (tfDetector != null) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0) {
                    telemetry.addData("Nada", "No results");
                } else {
                    for (Classifier.Recognition r : results) {
                        if (r.getConfidence() >= 0.8) {
                            if (r.getTitle().contains(LABEL_ZAYN)) {
                                this.result = LABEL_ZAYN;
                            }
                            else if(r.getTitle().contains(LABEL_AHMED)){
                                this.result = LABEL_AHMED;
                            }
                        }
                    }
                }
            }
            telemetry.update();
        }

    }


    public void initDetector() throws Exception {
        tfDetector = new Detector(MODEl_TYPE, getModelFileName(), getLabelFileName(), hardwareMap.appContext, telemetry);
    }

    protected void activateDetector() throws Exception {
        if (tfDetector != null) {
            tfDetector.activate();
        }
        telemetry.addData("Info", "TF Activated");
    }

    public void stopDetection() {
        stopThread();
        if (tfDetector != null) {
            tfDetector.stopProcessing();
        }
        tfDetector = null;
    }

    public void stopThread() {
        isRunning = false;
    }

    @Override
    public void run() {
        while(isRunning) {
            facialDetectionThread();
        }
    }

    public String getModelFileName() {
        return modelFileName;
    }

    public void setModelFileName(String modelFileName) {
        this.modelFileName = modelFileName;
    }

    public String getLabelFileName() {
        return labelFileName;
    }

    public void setLabelFileName(String labelFileName) {
        this.labelFileName = labelFileName;
    }

    public String getResult() {
        return result;
    }

    public void setResult(String result) {
        this.result = result;
    }
}
