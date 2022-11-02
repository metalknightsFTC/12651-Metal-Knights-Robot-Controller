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
    private DcMotor lift;
    private Servo grabber;
    private DriveTrainCode driveTrainCode;

    private int routine = 0;

    private IMUController imu;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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

    @Override
    public void runOpMode()
    {
        imu = new IMUController(hardwareMap);
        Expansion_Hub_1 = imu.Expansion_Hub_1;
        Expansion_Hub_2 = imu.Expansion_Hub_2;
        Initialize();
        waitForStart();
        SetLiftTarget(-1);
        CloseClaw();
        imu.ResetAngle();
        Move(48,0,1f);
        Move(0,48,1f);
        //CheckForModel();
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
        //SetRoutine();
        //RunRoutine();
    }

    //region From start to end position
    private void  RunRoutine() {
        switch (routine) {
            case 0:
                //region Red Left Case 0 Sword code
                RedLeftSideDropPreload();
                sleep(500);
                //move to parking
                Move(33,0f,.6f);
                sleep(500);
                Move(0,41f,.6f);
                //endregion
                break;
            case 1:
                //region Red Left Case 1 helmet code
                RedLeftSideDropPreload();
                sleep(500);
                Move(33,0f,.6f);
                sleep(500);
                Move(0,18f,.6f);
                //endregion
                break;
            case 2:
                //region Red Left Case 2 Shield code
                RedLeftSideDropPreload();
                sleep(500);
                Move(33,0f,.6f);
                sleep(500);
                //endregion
                break;
            case 3:
                //region Red Right 1 Sword code
                RedRightSideDropPreload();
                sleep(5000);
                //drive to position 1
                //endregion
                break;
            case 4:
                //region Red Right 2 Helmet code
                RedRightSideDropPreload();
                sleep(500);
                Move(10,0,.6f);
                sleep(500);
                Move(0,-24,.6f);
                //endregion
                break;
            case 5:
                //region Red Right 2 Shield code
                RedRightSideDropPreload();
                sleep(500);sleep(500);
                Move(10,0,.6f);
                sleep(500);
                Move(0,-40f,.6f);
                //endregion
                break;
            case 6:
                BlueLeftSideDropPreload();
                break;
            case 7:
                BlueLeftSideDropPreload();
                break;
            case 8:
                BlueLeftSideDropPreload();
                break;
            case 9:
                BlueRightSideDropPreload();
                Move(0,21,.6f);
                Move(30,0,.6f);
                break;
            case 10:
                BlueRightSideDropPreload();
                Move(0,21,.6f);
                Move(40,0,.6f);
                Move(0,-20,.6f);
                break;
            case 11:
                BlueRightSideDropPreload();
                Move(0,-21,.6f);
                Move(30,0,.6f);
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

    //region Lifter code
    void  SetLiftTarget(int level){
        int targetRotations = 0;

        switch (level) {
            case 0:
                targetRotations = (538 * 4);
                break;
            case 1:
                targetRotations = (int)(538 * 5.5);
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

    double coordZ = 0;
    double coordX = 0;

    //Drive x inches in selected direction at selected speed using odometry to record distance and the internal measuring unit(IMU) to correct for drift
    //slows down after a threshold is reached
    //region Movement code
    public void Move(float X, float Z, float speed){
        //how far the robot has left to move
        double xDist = X;
        double zDist = Z;
        //how far the robot has moved so far
        double totalMovementX = 0;
        double totalMovementZ = 0;
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
        int tpr = 8192;//found on REV encoder specs chart
        double c = 6.1575216; //Circumference of dead wheels (Math.PI) * (diameter / 2);//6.1575216

        float rampDown = 3;

        if(speed > .6f)
        {
            rampDown = 18;
        }

        //region X checks
        if(xDist != 0 && zDist == 0)
        {
            while (xDist < -.045f || xDist > .045f && opModeIsActive())
            {
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

                totalMovementZ += (deltaLeft + deltaRight) / 2;
                totalMovementX += deltaBack;

                xDist = X - totalMovementX;

                float delta = (float) (X - totalMovementX) / rampDown;
                delta = Range(delta,-1,1);

                float turnMod = (float) imu.AngleDeviation(0) / 20;

                telemetry.addData("Global Heading: ", imu.heading);
                telemetry.addData("Z: ", coordZ);
                telemetry.addData("X ", coordX);
                telemetry.update();

                driveTrainCode.UpdateDriveTrain(new Vector3(speed * delta, -turnMod*speed, 0));
            }
        }
        //endregion

        //region Z checks
        if(xDist == 0 && zDist != 0)
        {
            while (zDist < -.045f || zDist > .045f && opModeIsActive())
            {

                //region Odometry math
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

                totalMovementZ += (deltaLeft + deltaRight) / 2;
                totalMovementX += deltaBack;

                zDist = Z - totalMovementZ;

                float delta = (float) (Z - totalMovementZ) / rampDown;
                delta = Range(delta,-1,1);

                float turnMod = (float) imu.AngleDeviation(0) / 20;

                telemetry.addData("Global Heading: ", imu.heading);
                telemetry.addData("Z: ", coordZ);
                telemetry.addData("X ", coordX);
                telemetry.update();

                driveTrainCode.UpdateDriveTrain(new Vector3(0, -turnMod * speed, speed * delta));
            }
        }
        //endregion

        coordX += totalMovementX;
        coordZ += totalMovementZ;
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
        float deltaNC = (float) (target - imu.GetAngle());
        while (deltaNC > .04f || deltaNC < -.04f)
        {
            deltaNC = (float) (target - imu.GetAngle());
            float delta = (float) (target - imu.GetAngle()) / 3;

            driveTrainCode.UpdateDriveTrain(new Vector3(0, speed * delta, 0));
            telemetry.addData("CurrentHeading: ", imu.GetAngle());
            telemetry.update();
        }
        driveTrainCode.UpdateDriveTrain(new Vector3(0,0,0));
        imu.ResetAngle();
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

        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");
        rear = hardwareMap.get(DcMotorEx.class, "rear");

        driveTrainCode = new DriveTrainCode(gamepad1 ,hardwareMap);

        //driveTrainCode.InvertMotorDirection(Motor.frontLeft);
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

    private void RedLeftSideDropPreload()
    {
        //pushback from wall
        Move(3.5f,0f,.4f);
        //go to junction and lift
        sleep(500);
        Move(0,-15f,.5f);

        SetLiftTarget(0);
        sleep(500);

        Move(13.15f,0f,.5f);
        sleep(500);
        Move(0,3.1f,.4f);
        sleep(500);
        OpenClaw();
        sleep(500);

        Move(0,-4.7f,.4f);

        sleep(500);
        CloseClaw();
        sleep(500);
        SetLiftTarget(-1);
        sleep(500);
    }

    private void RedRightSideDropPreload()
    {
        //pull off of wall
        Move(3.5f,0,.4f);
        sleep(500);
        //drive to the entrypoint
        Move(0,25f,.5f);

        sleep(500);
        SetLiftTarget(2);
        sleep(500);

        Move(35.5f,0,.5f);
        sleep(500);
        Move(0,8.3f,.4f);
        OpenClaw();
        sleep(500);
        Move(0,-8.3f,.4f);

        sleep(500);
        SetLiftTarget(-1);
        sleep(500);

        CloseClaw();
        sleep(500);
    }

    private void BlueLeftSideDropPreload()
    {
        Move(3.5f,0,.4f);
        sleep(500);
    }

    private void BlueRightSideDropPreload()
    {
        Move(3.5f,0f,.4f);
        sleep(500);
        Move(0f,6.5f,.5f);
        sleep(500);
        SetLiftTarget(0);
        sleep(500);
        Move(12f,0f,.5f);
        sleep(500);
        Move(0f,3.5f,.4f);
        sleep(500);
        OpenClaw();
        sleep(500);
        Move(0,-4.5f,.4f);
        sleep(500);
        SetLiftTarget(-1);
        sleep(500);
        Move(-12,0,.5f);
    }
}