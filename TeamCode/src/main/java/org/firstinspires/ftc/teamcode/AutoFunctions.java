package org.firstinspires.ftc.teamcode;
import java.io.File;
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
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.SelectedDrive;
import org.firstinspires.ftc.teamcode.Enums.StartPoint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;

import android.annotation.SuppressLint;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.IOException;
import java.nio.file.FileSystem;
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
    private EndPoint endPoint = EndPoint.unset;
    private StartPoint startPoint = StartPoint.UnSet;
    DcMotor lift;
    Servo grabber;
    File test;
    DriveTrainCode driveTrainCode;
    int routine = 0;
    boolean routineComplete = false;

    double xInit;
    double tInit;
    double zInit;

    @SuppressLint("SdCardPath")
    private static final String TFOD_MODEL_ASSET =  /*"PowerPlay.tflite";*/
            "/sdcard/FIRST/tflitemodels/model.tflite";

    private static final String[] LABELS = {
            "shield",
            "helmet",
            "sword"
    };
    private static final String VUFORIA_KEY =
            "AWdhXNj/////AAABmRSQQCEQY0Z+t33w9GIgzFpsCMHl909n/+kfa54XDdq6fPjSi/8sBVItFQ/J/d5SoF48FrZl4Nz1zeCrwudfhFr4bfWTfh5oiLwKepThOhOYHf8V/GemTPe0+igXEu4VhznKcr3Bm5DiLe2b6zBVzvWFDWEHI/mkhLxRkU+llmwvitwodynP2arFgZ43thde9GJPCBFne/q6tPXeeN8/PoTUOtycTrnTkL6fBuHelMMnvN2RjqnMJ9SBUcaVX8DsWukq1fDr29O8bguAJU5JKxt9E3+XXiexpE/EJ9jxJc7YoMtpxfMro/e0sm9gRNckw4uPtZHnaoDjFhaK9t2D7kQQc3rwgK1OEZlY7FGQyy8g";;
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {
        lift = hardwareMap.get(DcMotor.class,"lifter");
        grabber = hardwareMap.get(Servo.class,"grabber");
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");

        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");
        rear = hardwareMap.get(DcMotorEx.class, "rear");

        driveTrainCode = new DriveTrainCode(gamepad1 ,hardwareMap);

        driveTrainCode.InvertMotorDirection(Motor.frontLeft);
        driveTrainCode.InvertMotorDirection(Motor.backRight);
        driveTrainCode.InvertMotorDirection(Motor.frontRight);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SetStartPoint();
        initVuforia();
        initTfod();

        InitializeOdometry();
        xInit = odometryControl.robotPosition.x;
        tInit = odometryControl.robotPosition.currentHeading;
        zInit = odometryControl.robotPosition.z;
        telemetry.addData("Path Set", ", End: ", endPoint, ", Start: ", startPoint);
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        waitForStart();


        OperateTensorFlow();
        UpdatePosition();

        SetRoutine();
        //end state as an integer value
        /*
        case 0 = red left 1
        case 1 = red left 2
        case 2 = red left 3

        case 3 = red right 1
        case 4 = red right 2
        case 5 = red right 3

        case 6 = blue left 1
        case 7 = blue left 2
        case 8 = blue left 3

        case 9 = blue right 1
        case 10 = blue right 2
        case 11 = blue right 3
         */

        //all things are done on x,z coordinate system with heading being in degrees
        RunRoutine();
    }

    public void  UpdateInit(){
        xInit = odometryControl.robotPosition.x;
        tInit = odometryControl.robotPosition.currentHeading;
        zInit = odometryControl.robotPosition.z;
    }

    //region From start to end position
    void  RunRoutine() {
        UpdateInit();
        switch (routine) {
            case 0:
                //z+1square
                //x+1square
                while (odometryControl.robotPosition.z <= zInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                routineComplete = true;
            case 1:
                //z+1
                //x+2
                //z-1
                while (odometryControl.robotPosition.z <= zInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit + 2){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                UpdateInit();
                while (odometryControl.robotPosition.z <= zInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                routineComplete = true;
            case 2:
                //z-1
                //x+1
                while (odometryControl.robotPosition.z <= zInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
            case 3:
                //z+1square
                //x+1square
                while (odometryControl.robotPosition.z <= zInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                routineComplete = true;
            case 4:
                //z+1
                //x+2
                //z-1
                while (odometryControl.robotPosition.z <= zInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit + 2){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                UpdateInit();
                while (odometryControl.robotPosition.z <= zInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                routineComplete = true;
            case 5:
                //z-1
                //x+1
                while (odometryControl.robotPosition.z <= zInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                routineComplete = true;
            case 6:
                //z-1
                //x-1
                while (odometryControl.robotPosition.z <= zInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                routineComplete = true;
            case 7:
                //z-1
                //x-2
                //z+1
                while (odometryControl.robotPosition.z <= zInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit - 2){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                UpdateInit();
                while (odometryControl.robotPosition.z <= zInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                routineComplete = true;
            case 8:
                //z+1
                //x-1
                while (odometryControl.robotPosition.z <= zInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                routineComplete = true;
            case 9:
                //z-1
                //x-1
                while (odometryControl.robotPosition.z <= zInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                routineComplete = true;
            case 10:
                //z-1
                //x-2
                //z+1
                while (odometryControl.robotPosition.z <= zInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit - 2){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                UpdateInit();
                while (odometryControl.robotPosition.z <= zInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                routineComplete = true;
            case 11:
                //z+1
                //x-1
                while (odometryControl.robotPosition.z <= zInit + 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
                }
                UpdateInit();
                while (odometryControl.robotPosition.x <= xInit - 1){
                    UpdatePosition();
                    driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
                }
                routineComplete = true;
            case 12:
                return;
            default:

        }

    }
    //endregion

    //region Set the routine to run
    void  SetRoutine(){
        switch (startPoint){

            case redLeft:
                switch (endPoint){
                    case one:
                        routine = 0;
                        break;
                    case two:
                        routine = 1;
                        break;
                    case three:
                        routine = 2;
                        break;
                    case unset:
                        routine = 12;
                        break;
                }
                break;
            case redRight:
                switch (endPoint){
                    case one:
                        routine = 3;
                        break;
                    case two:
                        routine = 4;
                        break;
                    case three:
                        routine = 5;
                        break;
                    case unset:
                        routine = 12;
                        break;
                }
                break;
            case blueLeft:
                switch (endPoint){
                    case one:
                        routine = 6;
                        break;
                    case two:
                        routine = 7;
                        break;
                    case three:
                        routine = 8;
                        break;
                    case unset:
                        routine = 12;
                        break;
                }
                break;
            case blueRight:
                switch (endPoint){
                    case one:
                        routine = 9;
                        break;
                    case two:
                        routine = 10;
                        break;
                    case three:
                        routine = 11;
                        break;
                    case unset:
                        routine = 12;
                        break;
                }
                break;
            case UnSet:
                break;
        }
    }
    //endregion

    void  UpdatePosition(){
        double[] deltas = odometryControl.CalculateRobotPosition();

        telemetry.addData("Right: ",right.getCurrentPosition());
        telemetry.addData("Left",left.getCurrentPosition());
        telemetry.addData("rear", rear.getCurrentPosition());

        odometryControl.robotPosition.currentHeading += deltas[2];
        odometryControl.robotPosition.x += deltas[0];
        odometryControl.robotPosition.z += deltas[1];
        telemetry.addData("X: ",odometryControl.robotPosition.x);
        telemetry.addData("Z: ",odometryControl.robotPosition.z);
        telemetry.addData("Heading: ",odometryControl.robotPosition.currentHeading);
        telemetry.update();
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
                        CheckEndPoint(recognition.getLabel());
                        i++;
                    }
                }
            }
        }
    }

    void CheckEndPoint(String label)
    {
        switch (label) {
            case "sword":
                //position 1
                endPoint = EndPoint.one;
                break;
            case "helmet":
                //position 2
                endPoint = EndPoint.two;
                break;
            case "shield":
                //position 3
                endPoint = EndPoint.three;
                break;
            default:
                endPoint = EndPoint.unset;
                return;
        }
        telemetry.addData("endPoint", endPoint.name());
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
        odometryControl = new OdometryControl(right, left, rear, new Vector3(0,0,0));
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
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        try {
            tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        }catch (Exception e){
            telemetry.addData("CV Status: ","RED NO MODEL FOUND");
            telemetry.update();
        }
        telemetry.addData("CV Status: ","GREEN");
    }



}