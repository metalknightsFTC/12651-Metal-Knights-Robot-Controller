package org.firstinspires.ftc.teamcode;
import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
import org.firstinspires.ftc.teamcode.Vectors.*;
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
    private static  WebcamName webcam2;
    private static SwitchableCamera switchableCamera;
    private int routine = 0;
    public static double targetHeading = 0;

    private IMUController imu;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public static int tpr = 8192;//found on REV encoder specs chart
    public static double c = 6.1575216; //Circumference of dead wheels (Math.PI) * (diameter / 2);//6.1575216
    public static float rampDown = 3f;

    Vector2 stackLocation;

    @SuppressLint("SdCardPath")
    private static String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/Stack.tflite";

    private static String[] LABELS = {"Blue Stack","Red Stack"};/*{
            "helmet",
            "shield",
            "sword"
    };*/

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
        while(opModeIsActive())
        {
            ScanForStack();
        }
        //CheckForModel();
        //SetRoutine();
        //RunRoutine();
    }

    //region Run the Autonomous
    private void  RunRoutine() {
        switch (routine) {
            case 0:
                //region Red Left Case 0 Sword code
                RedLeftSide();
                //move to parking
                sleep(100);
                Move(49,26f,.6f);
                //endregion
                break;

            case 1:
                //region Red Left Case 1 helmet code
                RedLeftSide();
                sleep(100);
                Move(49,2.7f,.6f);
                //endregion
                break;
            case 2:
                //region Red Left Case 2 Shield code
                RedLeftSide();
                //endregion
                break;
            case 3:
                //region Red Right 1 Sword code
                RedRightSide();
                sleep(5000);
                //drive to position 1
                //endregion
                break;
            case 4:
                //region Red Right 2 Helmet code
                RedRightSide();
                sleep(100);
                Move(49,3.5f,.4f);
                //endregion
                break;
            case 5:
                //region Red Right 2 Shield code
                RedRightSide();
                sleep(500);
                Move(49,-17,.4f);
                //endregion
                break;
            case 6:
                BlueLeftSide();
                Move(49,26.3f,.6f);
                break;
            case 7:
                BlueLeftSide();
                Move(49,3f,.6f);
                break;
            case 8:
                BlueLeftSide();
                break;
            case 9:
                BlueRightSide();
                break;
            case 10:
                BlueRightSide();
                sleep(100);
                Move(49,2,.4f);
                break;
            case 11:
                BlueRightSide();
                sleep(100);
                Move(49,-18f,.4f);
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
    double totalMovementX = 0;
    double totalMovementZ = 0;
    double errorMargin = .08f;
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
        if(speed > .6f)
        {
            rampDown = 20;
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

                driveTrainCode.UpdateDriveTrain(new Vector3(speed * deltaX, -turnMod * speed, speed * deltaZ));
        }
        //endregion
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        totalMovementX = 0;
        totalMovementZ = 0;
        driveTrainCode.UpdateDriveTrain(new Vector3(0,0,0));
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
        while (deltaNC > errorMargin || deltaNC < -errorMargin)
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
            telemetry.addData("X: ","Red Side Left");
            telemetry.addData("A: ","Red Side Right");

            telemetry.addData("Y: ","Blue Side Left");
            telemetry.addData("B: ","Blue Side Right");

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
            }
            if (gamepad1.y)
            {
                startPoint = StartPoint.blueLeft;
                return;
            }
            if (gamepad1.b)
            {
                startPoint = StartPoint.blueRight;
                return;
            }
        }
    }

    private void SetCameraAngle(double x, double z){
        verticalR.setPosition(z);
        horizontalR.setPosition(x);
    }

    private void OpenClaw(){
        grabber.setPosition(.13f);
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
        //tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        //tfod = null;
        //InitTfod();
        if(tfod!=null)
        {
            OperateStackScan();
        }
    }

    boolean OperateStackScan()
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
                    telemetry.addData("Left Bound: ",recognition.getLeft());
                    telemetry.addData("Right Bound: ",recognition.getRight());
                    telemetry.addData("Top Bound: ",recognition.getTop());
                    telemetry.addData("Bottom Bound: ",recognition.getBottom());
                    stackLocation = getDistance(recognition);
                    telemetry.addData("Distance: ", getDistance(recognition).toString());
                    telemetry.update();
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

    private Vector2 getDistance(Recognition recognition)
    {
        float x = 0;//recognition.getRight() - recognition.getLeft();
        float z = (-.45f * (recognition.getRight()- recognition.getLeft())) + 78.20f;
        return new Vector2(x,z);
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
        //pushback from wall
        Move(3.5f,0f,.4f);
        //go to junction and lift
        sleep(100);
        Move(0f,-14.4f,.4f);
        sleep(100);
        liftManager.Lift(5);
        sleep(100);
        Move(17f,0f,.4f);
        sleep(100);
        OpenClaw();
        sleep(100);
        Move(0f,-2f,.5f);
        sleep(100);
        liftManager.Lift(0);
        sleep(100);
        Move(32f,0f, .55f);
    }

    private void RedRightSide()
    {
        //pull off of wall
        Move(3.5f,0,.4f);
        sleep(100);
        //drive to the entrypoint
        Move(0f,28f,.5f);

        sleep(100);
        liftManager.Lift(7);
        sleep(100);

        Move(40f,0,.4f);
        sleep(100);
        Move(0f,3.5f,.4f);
        //sleep(100);
        OpenClaw();
        sleep(100);
        //Move(40.1f,28f,.4f);
        //sleep(100);
        sleep(10);
        Move(9f,-3.5f,.4f);
        sleep(50);
        liftManager.Lift(0);
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
        if(OperateStackScan())
        {
            liftManager.Lift(4);
            Move(stackLocation.x,stackLocation.z,.4f);
            CloseClaw();
            liftManager.Lift(5);
            sleep(1000);
        }
    }

}