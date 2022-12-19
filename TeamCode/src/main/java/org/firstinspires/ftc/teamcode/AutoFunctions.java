package org.firstinspires.ftc.teamcode;
import java.io.File;  // Import the File class
import java.io.FileNotFoundException;  // Import this class to handle errors
import java.util.Scanner;
import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors
import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Enums.EndPoint;
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.StartPoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.util.MandatoryWarningHandler;

import org.firstinspires.ftc.teamcode.Vectors.*;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoFunctions extends LinearOpMode {

    private Blinker Expansion_Hub_1;
    private Blinker Expansion_Hub_2;
    private DcMotorEx right;
    private DcMotorEx left;
    private DcMotorEx rear;
    private EndPoint endPoint = EndPoint.unset;
    private StartPoint startPoint = StartPoint.UnSet;
    private Servo grabber;
    private DriveTrainCode driveTrainCode;
    Servo verticalR;
    Servo horizontalR;
    private LiftManager liftManager;
    private static WebcamName webcam1;
    private int routine = 0;
    public static double targetHeading = 0;

    private IMUController imu;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public static int tpr = 8192;//found on REV encoder specs chart
    public static double c = 6.1575216; //Circumference of dead wheels (Math.PI) * (diameter / 2);//6.1575216
    public static float rampDown = 3f;
    public List<Float> averageBound = new ArrayList<>();
    float avgBound;
    int ii = 0;
    boolean problem = false;
    float offset = 3.5f;
    double errorMargin = .12f;
    Vector2 stackLocation;

    @SuppressLint("SdCardPath")
    private static String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model.tflite";

    private static String[] LABELS = {
            "helmet",
            "shield",
            "sword"
    };

    private static final String VUFORIA_KEY =
            "AWdhXNj/////AAABmRSQQCEQY0Z+t33w9GIgzFpsCMHl909n/+kfa54XDdq6fPjSi/8sBVItFQ/J/d5SoF48FrZl4Nz1zeCrwudfhFr4bfWTfh5oiLwKepThOhOYHf8V/GemTPe0+igXEu4VhznKcr3Bm5DiLe2b6zBVzvWFDWEHI/mkhLxRkU+llmwvitwodynP2arFgZ43thde9GJPCBFne/q6tPXeeN8/PoTUOtycTrnTkL6fBuHelMMnvN2RjqnMJ9SBUcaVX8DsWukq1fDr29O8bguAJU5JKxt9E3+XXiexpE/EJ9jxJc7YoMtpxfMro/e0sm9gRNckw4uPtZHnaoDjFhaK9t2D7kQQc3rwgK1OEZlY7FGQyy8g";

    @Override
    public void runOpMode()
    {
        liftManager = new LiftManager(hardwareMap);
        imu = new IMUController(hardwareMap);
        while(!imu.IMU_Calibrated()){
            telemetry.addData("IMU STATUS: ","Calibrating");
            telemetry.update();
        }
        Expansion_Hub_1 = imu.Expansion_Hub_1;
        Expansion_Hub_2 = imu.Expansion_Hub_2;
        Initialize();
        waitForStart();
        SetCameraAngle(0, .655);
        liftManager.Lift(0);
        CloseClaw();
        imu.ResetAngle();
        targetHeading = imu.GetAngle();
        CheckForModel();
        SetRoutine();
        RunRoutine();
        //GrabCone();
    }

    //region Run the Autonomous
    private void  RunRoutine() {
        switch (routine) {
            case 0:
                //region Red Left Case 0 Sword code
                RedLeftSide();
                //move to parking
                if(!problem) {
                    Move(0, 15.5f, .5f);
                }else
                {
                    Move(0, 40f, .5f);
                }
                //endregion
                break;

            case 1:
                //region Red Left Case 1 helmet code
                RedLeftSide();
                if(!problem) {
                    Move(0, -4f, .5f);
                    sleep(100);
                }else
                {
                    Move(0, 20f, .5f);
                }
                //endregion
                break;
            case 2:
                //region Red Left Case 2 Shield code
                RedLeftSide();
                if(!problem) {

                    Move(0, -28f, .58f);
                    sleep(100);
                }else
                {
                    Move(0, -4f, .5f);
                }
                //endregion
                break;
            case 3:
                //region Red Right 1 Sword code
                RedRightSide();
                //drive to position 1
                //endregion
                break;
            case 4:
                //region Red Right 2 Helmet code
                RedRightSide();
                Move(0,-21f,.58f);
                //endregion
                break;
            case 5:
                //region Red Right 2 Shield code
                RedRightSide();
                Move(0,-42.9f,.58f);
                //SnapToHeading(90,.58f);
                //endregion
                break;
            case 6:
                RedLeftSide();
                break;
            case 7:
                RedLeftSide();
                break;
            case 8:
                RedLeftSide();
                break;
            case 9:
                RedRightSide();
                break;
            case 10:
                RedRightSide();
                break;
            case 11:
                RedRightSide();
                break;
            default:
                break;
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

    //Drive x inches in selected direction at selected speed using odometry to record distance and the internal measuring unit(IMU) to correct for drift
    //slows down after a threshold is reached
    //region Movement code
    //the total amount the robot has moved
    int to = 0;
    double totalMovementX = 0;
    double totalMovementZ = 0;
    public void Move(float X, float Z, float speed)
    {
        //how far the robot has left to move
        double xDist = X;
        double zDist = Z;
        //how far the robot has moved so far
        //previously measured encoder positions
        double rearEncoderRotation;
        double rightEncoderRotation;
        double leftEncoderRotation;
        //currently measured encoder rotation
        double currentLeftEncoderRotation = 0;
        double currentRightEncoderRotation = 0;
        double currentRearEncoderRotation = 0;
        //the distance between the current encoder positions and the previous encoder positions
        double deltaLeft;
        double deltaRight;
        double deltaBack;
        //ticks per revolution
        if(speed > .8f)
        {
            rampDown = 20f;
        }else if(speed > .6f)
        {
            rampDown = 12.5f;
        }else
        {
            rampDown = 2.75f;
        }
        //region checks
        while (xDist < -errorMargin || xDist > errorMargin || zDist < -errorMargin || zDist > errorMargin && opModeIsActive()) {
                //region Odometry Math
                leftEncoderRotation = currentLeftEncoderRotation;
                rightEncoderRotation = currentRightEncoderRotation;
                rearEncoderRotation = currentRearEncoderRotation;

                currentRightEncoderRotation = right.getCurrentPosition();
                currentRearEncoderRotation = rear.getCurrentPosition();
                currentLeftEncoderRotation = left.getCurrentPosition();

                deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / tpr) * c;
                deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / tpr) * c;
                deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / tpr) * c;
                //endregion

                totalMovementZ += ((deltaLeft + deltaRight) / 2);
                totalMovementX += deltaBack;

                xDist = X - totalMovementX;
                zDist = Z - totalMovementZ;

                float deltaX = (float) (X - totalMovementX) / rampDown;
                float deltaZ = (float) (Z - totalMovementZ) / rampDown;
                deltaX = Range(deltaX, -1, 1);
                deltaZ = Range(deltaZ, -1, 1);
                float turnMod = (float) imu.AngleDeviation(targetHeading) / 20;

                telemetry.addData("Global Heading: ", imu.heading);
                telemetry.addData("Z: ", zDist);
                telemetry.addData("X ", xDist);
                telemetry.update();
                float x = speed * deltaX;
                float y = -turnMod * speed;
                float z = speed * deltaZ;
                float distX = (float)Math.pow(deltaX,3f);
                float distZ = (float)Math.pow(deltaZ,3f);

                if((distX < .03f && distX > -.03f) && (distZ < .03f && distZ > -.03f))
                {
                    break;
                }

                driveTrainCode.UpdateDriveTrain(new Vector3(x, y, z));
        }
        //endregion
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        totalMovementX = 0;
        totalMovementZ = 0;
        driveTrainCode.UpdateDriveTrain(new Vector3(0,0,0));
        to = 0;
    }
    //endregion

    //region TensorFlow code
    boolean OperateTensorFlow()
    {
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("Label: ", recognition.getLabel());
                        CheckEndPoint(recognition.getLabel());
                        return true;
                    }
                }else
                {
                    return false;
                }
        }else
        {
            return false;
        }
        return  false;
    }
    //endregion

    //region lock input between 2 numbers
    private float Range(float in,float lower, float upper){
        float out = in;
        if(out < lower){
            out = lower;
        }
        if(out > upper){
            out = upper;
        }
        return out;
    }
    //endregion

    private void SnapToHeading(float target, float speed){
        float deltaNC = (float) (imu.GetAngle() - target);
        while (deltaNC > .1f || deltaNC < -.1f)
        {
            deltaNC = (float) (imu.GetAngle()-target);
            float delta = (float) deltaNC / 20;

            driveTrainCode.UpdateDriveTrain(new Vector3(0, speed * delta, 0));
            telemetry.addData("CurrentHeading: ", imu.GetAngle());
            telemetry.update();
        }
        driveTrainCode.UpdateDriveTrain(new Vector3(0,0,0));
        targetHeading = imu.GetAngle();
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                OperateTensorFlow();
        }
        telemetry.addData("endPoint", endPoint.name());
    }

    void  SetStartPoint()
    {
        while(startPoint == StartPoint.UnSet && !isStopRequested())
        {
            telemetry.addData("X: ","Left");
            telemetry.addData("A: ","Right");

            //telemetry.addData("Y: ","Blue Side Left");
            //telemetry.addData("B: ","Blue Side Right");

            telemetry.update();

            if (gamepad1.a)
            {
                startPoint = StartPoint.redRight;
                return;
            }
            if (gamepad1.x)
            {
                startPoint = StartPoint.redLeft;
                return;
            }/*
            if (gamepad1.y)
            {
                startPoint = StartPoint.blueLeft;
                return;
            }
            if (gamepad1.b)
            {
                startPoint = StartPoint.blueRight;
                return;
            }*/
        }
    }

    private void SetCameraAngle(double x, double z){
        verticalR.setPosition(z);
        horizontalR.setPosition(x);
    }

    private void OpenClaw(){
        grabber.setPosition(.145f);
    }

    private void CloseClaw(){
        grabber.setPosition(.33f);
    }

    private void InitVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        //webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        //parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
        parameters.cameraName = webcam1;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Set the active camera to Webcam 1.
        //switchableCamera = (SwitchableCamera) vuforia.getCamera();
        //if(switchableCamera!=null)
        //{
        //    switchableCamera.setActiveCamera(webcam1);
        //}
    }

    private void InitTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        try {
            tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        }catch (Exception e){
            telemetry.addData("CV Status: ","RED NO MODEL FOUND");
            telemetry.update();
        }
        telemetry.addData("CV Status: ","GREEN");
    }

    @SuppressLint("SdCardPath")
    private void ScanForStack(){
        SetCameraAngle(.04,.27);
        LABELS = new String[] {"Blue Stack","Red Stack"};
        TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/Stack.tflite";
        tfod.shutdown();
        tfod = null;
        InitTfod();
        if(tfod!=null)
        {
            while (!OperateStackScan()&&opModeIsActive())
            {
                telemetry.addData("Scanning...","");
            }
        }
    }

    boolean OperateStackScan()
    {
        SetCameraAngle(.04,.27);
        if (tfod != null)
        {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null)
            {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
                if(updatedRecognitions.size() == 0)
                {
                    to++;
                    return false;
                }
                if(to >= 50)
                {
                    problem = true;
                    return true;
                }
                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : updatedRecognitions)
                {
                    telemetry.addData("Label: ", recognition.getLabel());
                    telemetry.addData("Left Bound: ",recognition.getLeft());
                    telemetry.addData("Right Bound: ",recognition.getRight());
                    telemetry.addData("Top Bound: ",recognition.getTop());
                    telemetry.addData("Bottom Bound: ",recognition.getBottom());
                    telemetry.update();
                    if(recognition.getLabel().equals("Red Stack"))
                    {
                        offset = 3.5f;
                    }else if(recognition.getLabel().equals("Blue Stack"))
                    {
                        offset = 1.1f;
                    }
                    if(ii <= 20)
                    {
                        averageBound.add(recognition.getRight()-recognition.getLeft());
                        ii++;
                        return false;
                    }
                    for (int iii = 0; iii < ii; iii++){
                        avgBound += averageBound.get(iii);
                    }
                    avgBound /= ii;
                    stackLocation =
                            getDistance(avgBound,recognition.getRight() + recognition.getLeft());
                    return true;
                }
            }else
            {
                return false;
            }
        }else
        {
            return false;
        }
        return  false;
    }

    boolean xScan()
    {
        SetCameraAngle(.04,.27);
        if (tfod != null)
        {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null)
            {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : updatedRecognitions)
                {
                    telemetry.addData("Label: ", recognition.getLabel());
                    telemetry.addData("Left Bound: ",recognition.getLeft());
                    telemetry.addData("Right Bound: ",recognition.getRight());
                    telemetry.addData("Top Bound: ",recognition.getTop());
                    telemetry.addData("Bottom Bound: ",recognition.getBottom());
                    telemetry.update();
                    stackLocation =
                            getDistance(avgBound,recognition.getRight() + recognition.getLeft());
                    return true;
                }
            }else
            {
                return false;
            }
        }else
        {
            return false;
        }
        return  false;
    }


    private Vector2 getDistance(float deltaBound, float bound)
    {
        float x = (-0.0586f * ((bound) / 2)) + 21.834f;
        // linear
        // float z = (-0.35f * (deltaBound)) + 65.85f;
        //exponential
        //float z = (float) (94.3f * Math.exp(-0.0118f * deltaBound));
        //polynomial
        float z = (float)(108f + ((-1.28f * deltaBound) + (0.00462f * Math.pow(deltaBound,2))));

        return new Vector2(x,z + offset);
    }

    private void Initialize()
    {

        horizontalR = hardwareMap.get(Servo.class,"alignment");
        verticalR = hardwareMap.get(Servo.class,"pivot");
        grabber = hardwareMap.get(Servo.class,"grabber");

        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");
        rear = hardwareMap.get(DcMotorEx.class, "rear");

        driveTrainCode = new DriveTrainCode(gamepad1 ,hardwareMap);

        driveTrainCode.InvertMotorDirection(Motor.backRight);
        driveTrainCode.InvertMotorDirection(Motor.frontRight);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        rear.setDirection(DcMotorSimple.Direction.REVERSE);

        SetStartPoint();
        InitVuforia();
        InitTfod();

        if(startPoint == StartPoint.redLeft){
            telemetry.addData("Path Set ", "Start: Red Left");
        }else if(startPoint == StartPoint.redRight){
            telemetry.addData("Path Set ", "Start: Red Right");
        }else if(startPoint == StartPoint.blueLeft){
            telemetry.addData("Path Set ", "Start: Blue Left");
        }else if(startPoint == StartPoint.blueRight){
            telemetry.addData("Path Set ", "Start: Blue Right");
        }

        telemetry.addData("IMU STATUS: ","Calibrated");
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

    }
    
    private void CheckForModel()
    {
        while (!OperateTensorFlow()){
            telemetry.addData("Scanning",".");
            telemetry.update();
            if(OperateTensorFlow())
                return;
            telemetry.addData("Scanning","..");
            telemetry.update();
            if(OperateTensorFlow())
                return;
            telemetry.addData("Scanning","...");
            telemetry.update();
            if(OperateTensorFlow())
                return;
            telemetry.addData("Scanning","....");
            telemetry.update();
            if(OperateTensorFlow())
                return;
            telemetry.addData("Scanning",".....");
            telemetry.update();
        }
    }

    private void RedLeftSide()
    {
        SetCameraAngle(.04,.27);
        //pushback from wall
        Move(3.5f,0f,.45f);
        //go to junction and lift
        sleep(100);
        Move(0f,-14.4f,.45f);
        sleep(100);
        liftManager.Lift(5);
        sleep(100);
        Move(14f,0f,.45f);
        sleep(100);
        OpenClaw();
        sleep(100);
        Move(0f,-2f,.45f);
        sleep(100);
        liftManager.Lift(0);
        sleep(100);
        Move(34f,0f, .65f);
        sleep(100);
        Move(0,4f,.45f);
        sleep(100);
        GrabCone();
    }

    private void RedRightSide()
    {
        SetCameraAngle(.04,.27);
        //pull off wall
        Move(3.5f,0f,.4f);
        sleep(100);
        //move up to junction
        Move(0f,7.2f,.5f);
        sleep(100);
        //lift
        liftManager.Lift(5);
        sleep(100);
        //move on top of junction
        Move(14f,0f,.5f);
        sleep(100);
        //drop cone
        OpenClaw();
        sleep(100);
        // pull back from junction
        Move(0f,-2.3f,.4f);
        sleep(100);
        //strafe back to center of the path
        Move(-14.6f,0f,.5f);
        sleep(100);
        liftManager.Lift(0);
        sleep(100);
        //drive up to prepare for strafe
        Move(0f,21.5f,.4f);
        sleep(100);
        //strafe to line up on stack
        Move(46.5f,0f,.65f);
        sleep(100);
        //turn to stack
        //SnapToHeading(180,.4f);
        //grab and drop
        //GrabCone();
    }

    private void BlueLeftSide()
    {
        //pushback from wall
        Move(3.5f,0f,.4f);
        //go to junction and lift
        sleep(100);
        Move(0f,-14.3f,.4f);
        sleep(100);
        liftManager.Lift(5);
        sleep(100);
        Move(16.7f,0f,.4f);
        sleep(100);
        OpenClaw();
        sleep(100);
        Move(0,-2.7f,.5f);
        sleep(100);
        liftManager.Lift(0);
        sleep(100);
        Move(47.8f,2.7f, .55f);
    }

    private void BlueRightSide()
    {
        Move(3.5f,0f,.4f);
        sleep(100);
        Move(0f,7.2f,.5f);
        sleep(100);
        liftManager.Lift(5);
        sleep(100);
        Move(14.6f,0f,.5f);
        sleep(100);
        OpenClaw();
        sleep(100);
        Move(0f,2.3f,.4f);
        sleep(100);
        Move(-14.6f,0f,.5f);
        sleep(100);
        liftManager.Lift(0);
        sleep(100);
        Move(0f,19.5f,.4f);
        sleep(100);
        Move(49,0f,.5f);
    }

    private void GrabCone()
    {
        OpenClaw();
        sleep(100);
        ScanForStack();
        if(!problem)
        {
            liftManager.Lift(4);
            telemetry.addData("Position: ", stackLocation.toString());
            telemetry.update();
            sleep(100);
            Move(-stackLocation.x, 0, .4f);
            sleep(100);
            Move(0, stackLocation.z, .4f);
            sleep(100);
            CloseClaw();
            sleep(750);
            liftManager.Lift(5);
            sleep(100);
            Move(0, -(stackLocation.z - 22.5f), .45f);
            sleep(100);
            //liftManager.Lift(0);
            DropConeLeft();
            sleep(100);
            Move(0, (stackLocation.z - 22.5f), .45f);
            sleep(100);
            Move(0, -(stackLocation.z - 22.5f), .45f);
            DropConeLeft();
        }
    }

    private void DropConeLeft()
    {
        Move(-9.75f,0,.45f);
        sleep(100);
        OpenClaw();
        sleep(100);
        Move(9f,0,.45f);
        sleep(100);
        liftManager.Lift(4);
        sleep(100);
        //xScan();
        //Move(-stackLocation.x,0,.55f);
    }

}