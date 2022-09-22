package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Enums.EndPoint;
import org.firstinspires.ftc.teamcode.Enums.StartPoint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class AutoFunctions extends LinearOpMode {

    Blinker Expansion_Hub_1;
    Blinker Expansion_Hub_2;
    private DcMotorEx right;
    private DcMotorEx left;
    private DcMotorEx rear;
    OdometryControl odometryControl;
    private EndPoint endPoint;
    private StartPoint startPoint = StartPoint.UnSet;
    private SubSystemControl subSystemControl;
    String lift = "Lift";
    String grabber = "Grabber";

    String[] motors = new String[1];
    String[] servos = new String[1];

    private static final String TFOD_MODEL_ASSET =  "PowerPlay.tflite";//"/sdcard/FIRST/tflitemodels/model.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String VUFORIA_KEY =
            "AWdhXNj/////AAABmRSQQCEQY0Z+t33w9GIgzFpsCMHl909n/+kfa54XDdq6fPjSi/8sBVItFQ/J/d5SoF48FrZl4Nz1zeCrwudfhFr4bfWTfh5oiLwKepThOhOYHf8V/GemTPe0+igXEu4VhznKcr3Bm5DiLe2b6zBVzvWFDWEHI/mkhLxRkU+llmwvitwodynP2arFgZ43thde9GJPCBFne/q6tPXeeN8/PoTUOtycTrnTkL6fBuHelMMnvN2RjqnMJ9SBUcaVX8DsWukq1fDr29O8bguAJU5JKxt9E3+XXiexpE/EJ9jxJc7YoMtpxfMro/e0sm9gRNckw4uPtZHnaoDjFhaK9t2D7kQQc3rwgK1OEZlY7FGQyy8g";;
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {

        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        //Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub");

        right = hardwareMap.get(DcMotorEx.class, "frontRight");
        left = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rear = hardwareMap.get(DcMotorEx.class, "backRight");

        motors[0] = lift;
        servos[0] = grabber;

        subSystemControl = new SubSystemControl(hardwareMap, motors, servos);

        initVuforia();
        initTfod();
        //SetStartPoint();
        //InitializeOdometry();
        telemetry.addData("Path Set", ", End: ", endPoint, ", Start: ", startPoint);
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            OperateTensorFlow();
            /*
            double[] deltas = odometryControl.CalculateRobotPosition();

            odometryControl.robotPosition.currentHeading += deltas[2];
            if (deltas[2] <= .2f || deltas[2] >= -.2f)
            {
                odometryControl.robotPosition.x += deltas[0];
            }
            odometryControl.robotPosition.z += deltas[1];
            telemetry.addData("X: ",odometryControl.robotPosition.x);
            telemetry.addData("Z: ",odometryControl.robotPosition.z);
            telemetry.addData("Heading: ",odometryControl.robotPosition.currentHeading);
            telemetry.update();
            */
        }
    }

    void OperateTensorFlow()
    {
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                }
            }
        }
    }

    void  SetStartPoint(){
        while(startPoint == StartPoint.UnSet)
        {
            telemetry.addData("A: ","Red Side Left");
            telemetry.addData("X: ","Red Side Right");
            telemetry.addData("Y: ","Blue Side Left");
            telemetry.addData("B: ","Blue Side Right");
            telemetry.update();
            if (gamepad1.a){
                startPoint = StartPoint.redLeft;
                return;
            }
            if (gamepad1.x){
                startPoint = StartPoint.redRight;
                return;
            }
            if (gamepad1.y){
                startPoint = StartPoint.blueLeft;
                return;
            }
            if (gamepad1.b){
                startPoint = StartPoint.blueRight;
                return;
            }
        }
    }

    void InitializeOdometry(){
        odometryControl = new OdometryControl(right, left, rear, hardwareMap, gamepad1);
        telemetry.addData("Odometry Status: ", "Ready");
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        //tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }

}