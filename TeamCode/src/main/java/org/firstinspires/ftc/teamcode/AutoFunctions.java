package org.firstinspires.ftc.teamcode;
import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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
import java.util.List;

@Autonomous
public class AutoFunctions extends LinearOpMode {

    Blinker Expansion_Hub_1;
    Blinker Expansion_Hub_2;
    private DcMotorEx right;
    private DcMotorEx left;
    private DcMotorEx rear;
    private EndPoint endPoint = EndPoint.unset;
    private StartPoint startPoint = StartPoint.UnSet;
    DcMotor lift;
    Servo grabber;
    DriveTrainCode driveTrainCode;

    int routine = 0;

    int cpr = 8192;
    double c = 6.1575216;//(Math.PI) * (diameter / 2);//6.1575216

    double currentLeftEncoderRotation = 0;
    double currentRightEncoderRotation = 0;
    double currentRearEncoderRotation = 0;
    double leftEncoderRotation = 0;
    double rightEncoderRotation = 0;
    double rearEncoderRotation = 0;

    @SuppressLint("SdCardPath")
    private static final String TFOD_MODEL_ASSET =  /*"PowerPlay.tflite";*/
            "/sdcard/FIRST/tflitemodels/model.tflite";

    private static final String[] LABELS = {
            "helmet",
            "shield",
            "sword"
    };
    private static final String VUFORIA_KEY =
            "AWdhXNj/////AAABmRSQQCEQY0Z+t33w9GIgzFpsCMHl909n/+kfa54XDdq6fPjSi/8sBVItFQ/J/d5SoF48FrZl4Nz1zeCrwudfhFr4bfWTfh5oiLwKepThOhOYHf8V/GemTPe0+igXEu4VhznKcr3Bm5DiLe2b6zBVzvWFDWEHI/mkhLxRkU+llmwvitwodynP2arFgZ43thde9GJPCBFne/q6tPXeeN8/PoTUOtycTrnTkL6fBuHelMMnvN2RjqnMJ9SBUcaVX8DsWukq1fDr29O8bguAJU5JKxt9E3+XXiexpE/EJ9jxJc7YoMtpxfMro/e0sm9gRNckw4uPtZHnaoDjFhaK9t2D7kQQc3rwgK1OEZlY7FGQyy8g";
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode()
    {
        Initialize();
        waitForStart();
        SetLiftTarget(-1);
        CloseClaw();
        CheckForModel();
        //region end state as an integer value
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
        //endregion
        SetRoutine();

        RunRoutine();
    }

    //region From start to end position
    private void  RunRoutine() {
        switch (routine) {
            case 0:
                //region Case 0 Sword code
                LeftSideDropPreload();
                sleep(500);
                //move to parking
                Move(31,0f,0f,.45f);
                Move(0,36.5f,0,.45f);
                //Move(12,0,0,.45f);
                break;
            case 1:
                //region Case 1 helmet code
                LeftSideDropPreload();
                sleep(500);
                Move(31,0f,0f,.45f);
                Move(0,20f,0,.45f);
                //endregion
                break;
            case 2:
                //region Case 2 Shield code
                LeftSideDropPreload();
                Move(5,0f,0f,.45f);
                //endregion
                break;
            case 3:
                RightSideDropPreload();
                //drive to position 1
                break;
            case 4:
                RightSideDropPreload();
                //drive to position 2
                Move(7.5f,0,0,.45f);
                Move(0,-17,0,.45f);
                break;
            case 5:
                RightSideDropPreload();
                //drive to position 3
                Move(7,0,0,.45f);
                Move(0,-37,0,.45f);
                break;
            default:
                break;
        }
    }
    //endregion

    //region Set the routine to run
    void  SetRoutine(){
        switch (startPoint){

            case left:
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
            case right:
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
                break;/*
            case blueLeft:
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
            case blueRight:
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
                break;*/
            case UnSet:
                break;
        }
    }
    //endregion

    //region Lifter code
    void  SetLiftTarget(int level){
        int targetRotations = 0;

        switch (level) {
            case 0:
                targetRotations = (int) (538 * 4);
                break;
            case 1:
                targetRotations = (int) (538 * 5.5);
                break;
            case 2:
                targetRotations = 4650;
                break;
            default:
                break;
        }

        lift.setTargetPosition(targetRotations);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.65);
    }
//endregion

    //region Movement code
    public void Move(float X, float Z,float thetaR, float speed){
        double totalTurnCircumference = 4.925;
        double theta = totalTurnCircumference/thetaR;
        double totalMovementX = 0;
        double totalMovementZ = 0;
        double t = 0;
        double deltaLeft;
        double deltaRight;
        double deltaBack;
        //region X checks
        if(X > 0 && Z == 0) {
            while (totalMovementX < X && opModeIsActive()) {
                leftEncoderRotation = currentLeftEncoderRotation;
                rightEncoderRotation = currentRightEncoderRotation;
                rearEncoderRotation = currentRearEncoderRotation;

                currentRightEncoderRotation = right.getCurrentPosition();
                currentRearEncoderRotation = rear.getCurrentPosition();
                currentLeftEncoderRotation = left.getCurrentPosition();

                deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / cpr) * c;
                deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / cpr) * c;
                deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / cpr) * c;

                totalMovementZ += (deltaLeft + deltaRight) / 2;
                totalMovementX += deltaBack;

                telemetry.addData("X ", totalMovementX);
                telemetry.update();
                driveTrainCode.UpdateDriveTrain(new Vector3(speed, 0, 0));
            }
        }
        if(X < 0 && Z == 0) {
            while (totalMovementX > X && opModeIsActive()) {
                leftEncoderRotation = currentLeftEncoderRotation;
                rightEncoderRotation = currentRightEncoderRotation;
                rearEncoderRotation = currentRearEncoderRotation;

                currentRightEncoderRotation = right.getCurrentPosition();
                currentRearEncoderRotation = rear.getCurrentPosition();
                currentLeftEncoderRotation = left.getCurrentPosition();

                deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / cpr) * c;
                deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / cpr) * c;
                deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / cpr) * c;

                totalMovementZ += (deltaLeft + deltaRight) / 2;
                totalMovementX += deltaBack;

                telemetry.addData("X: ", totalMovementX);
                telemetry.update();
                driveTrainCode.UpdateDriveTrain(new Vector3(-speed, 0, 0));
            }
        }
        //endregion

        //region Z checks
        if(X == 0 && Z > 0) {
            while (totalMovementZ < Z && opModeIsActive()) {
                leftEncoderRotation = currentLeftEncoderRotation;
                rightEncoderRotation = currentRightEncoderRotation;
                rearEncoderRotation = currentRearEncoderRotation;

                currentRightEncoderRotation = right.getCurrentPosition();
                currentRearEncoderRotation = rear.getCurrentPosition();
                currentLeftEncoderRotation = left.getCurrentPosition();

                deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / cpr) * c;
                deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / cpr) * c;
                deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / cpr) * c;

                totalMovementZ += (deltaLeft + deltaRight) / 2;
                totalMovementX += deltaBack;

                telemetry.addData("Z: ", totalMovementZ);
                telemetry.update();
                driveTrainCode.UpdateDriveTrain(new Vector3(0, 0, speed));
            }
        }
        if(X == 0 && Z < 0) {
            while (totalMovementZ > Z && opModeIsActive()) {
                leftEncoderRotation = currentLeftEncoderRotation;
                rightEncoderRotation = currentRightEncoderRotation;
                rearEncoderRotation = currentRearEncoderRotation;

                currentRightEncoderRotation = right.getCurrentPosition();
                currentRearEncoderRotation = rear.getCurrentPosition();
                currentLeftEncoderRotation = left.getCurrentPosition();

                deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / cpr) * c;
                deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / cpr) * c;
                deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / cpr) * c;

                totalMovementZ += (deltaLeft + deltaRight) / 2;
                totalMovementX += deltaBack;

                telemetry.addData("Z: ", totalMovementZ);
                telemetry.update();
                driveTrainCode.UpdateDriveTrain(new Vector3(0, 0, -speed));
            }
        }
        //endregion

        //region Turn Checks

        if(X == 0 && Z == 0 && theta > 0){
            while (t < theta && opModeIsActive())
            {
                leftEncoderRotation = currentLeftEncoderRotation;
                rightEncoderRotation = currentRightEncoderRotation;
                rearEncoderRotation = currentRearEncoderRotation;

                currentRightEncoderRotation = right.getCurrentPosition();
                currentRearEncoderRotation = rear.getCurrentPosition();
                currentLeftEncoderRotation = left.getCurrentPosition();

                deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / cpr) * c;
                deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / cpr) * c;
                deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / cpr) * c;

                totalMovementZ += (deltaLeft + deltaRight) / 2;
                totalMovementX += deltaBack;
                t += (deltaLeft-deltaRight)/2;
                telemetry.addData("Turn: ", t);
                telemetry.update();
                driveTrainCode.UpdateDriveTrain(new Vector3(0, speed, 0));
            }
        }

        if(X == 0 && Z == 0 && theta < 0){
            while (t > theta && opModeIsActive())
            {
                leftEncoderRotation = currentLeftEncoderRotation;
                rightEncoderRotation = currentRightEncoderRotation;
                rearEncoderRotation = currentRearEncoderRotation;

                currentRightEncoderRotation = right.getCurrentPosition();
                currentRearEncoderRotation = rear.getCurrentPosition();
                currentLeftEncoderRotation = left.getCurrentPosition();

                deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / cpr) * c;
                deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / cpr) * c;
                deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / cpr) * c;

                totalMovementZ += (deltaLeft + deltaRight) / 2;
                totalMovementX += deltaBack;
                t += (deltaLeft-deltaRight)/2;
                telemetry.addData("Turn: ", t);
                telemetry.update();
                driveTrainCode.UpdateDriveTrain(new Vector3(0, -speed, 0));
            }
        }

        //endregion
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
            telemetry.addData("B: ","Right");
            /*
            telemetry.addData("Y: ","Blue Side Left");
            telemetry.addData("B: ","Blue Side Right");
            */
            telemetry.update();
            /*
            if (gamepad1.a)
            {
                startPoint = StartPoint.left;
                return;
            }
            */
            if (gamepad1.x)
            {
                startPoint = StartPoint.left;
                return;
            }
            /*
            if (gamepad1.y)
            {
                startPoint = StartPoint.blueLeft;
                return;
            }
            */
            if (gamepad1.b)
            {
                startPoint = StartPoint.right;
                return;
            }
        }
    }

    private void OpenClaw(){
        grabber.setPosition(.22f);
    }

    private void CloseClaw(){
        grabber.setPosition(.32f);
    }

    private void InitVuforia()
    {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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

    private void Initialize()
    {
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
        lift.setDirection(DcMotorSimple.Direction.REVERSE);


        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        rear.setDirection(DcMotorSimple.Direction.REVERSE);

        SetStartPoint();
        InitVuforia();
        InitTfod();

        if(startPoint == StartPoint.left){
            telemetry.addData("Path Set ", "Start: Left");
        }else if(startPoint == StartPoint.right){
            telemetry.addData("Path Set ", "Start: Right");
        }
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

    private void LeftSideDropPreload()
    {
        //pushback from wall
        Move(1.5f,0f,0f,.3f);
        //go to junction and lift
        Move(0,-14f,0f,.45f);
        SetLiftTarget(0);
        sleep(500);
        Move(12,0f,0f,.45f);
        Move(0,3f,0f,.25f);
        sleep(500);
        OpenClaw();
        sleep(500);
        //back off junction
        Move(0,-2.5f,0f,.25f);
        CloseClaw();
        SetLiftTarget(-1);
        sleep(500);
    }

    private void RightSideDropPreload()
    {
        //pull off of wall
        Move(1.5f,0,0,.3f);
        //drive to the entrypoint
        Move(0,15,0,.45f);
        //Lift
        SetLiftTarget(2);
        sleep(500);
        //Drive to High junction
        Move(33,0,0,.45f);
        //drive on top of high Junction
        Move(0,3,0,.45f);
        //drop cone
        OpenClaw();
        sleep(500);
        //move off of junction
        Move(0,-2,0,.45f);
        //lower lift
        SetLiftTarget(-1);
        CloseClaw();
        sleep(500);
    }


}