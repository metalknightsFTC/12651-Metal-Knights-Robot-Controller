package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Enums.EndPoint;
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.SelectedDrive;
import org.firstinspires.ftc.teamcode.Enums.StartPoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
                //pushback from wall
                Move(1.5f,0f,0f,.3f);
                //turn to junction
                Move(0,0f,1.93f,.45f);
                //lift
                SetLiftTarget(0);
                //wait to lift
                sleep(1000);
                //drive to the junction
                Move(0f,9f,0f,.3f);
                //wait a bit to be sure
                sleep(1000);
                //drop cone
                OpenClaw();
                sleep(1000);
                //move back off of junction9
                Move(0f,-3f,0f,.3f);
                //lower
                SetLiftTarget(-1);
                Move(0,0f,-3.91f,.45f);
                //strafe into position
                Move(-1f,0f,0f,.4f);
                //move to position 1
                Move(0f,22,0f,.4f);
                Move(24,0,0,.4f);
                //endregion
                break;
            case 1:
                //region Case 1 helmet code
                //pushback from wall
                Move(1.5f,0f,0f,.3f);
                //turn to junction
                Move(0,0f,1.93f,.45f);
                //lift
                SetLiftTarget(0);
                //wait to lift
                sleep(1000);
                //drive to the junction
                Move(0f,9f,0f,.3f);
                //wait a bit to be sure
                sleep(1000);
                //drop cone
                OpenClaw();
                sleep(1000);
                //move back off of junction9
                Move(0f,-3f,0f,.3f);
                //lower
                SetLiftTarget(-1);
                Move(0,0f,-3.92f,.45f);
                //strafe into position
                Move(-1f,0f,0f,.4f);
                Move(0,1.8f,0,.4f);
                //move to position 2
                Move(20,0f,0f,.5f);
                //endregion
                break;
            case 2:
                //region Case 2 Shield code
                //pushback from wall
                Move(1.5f,0f,0f,.3f);
                //turn to junction
                Move(0,0f,1.93f,.45f);
                //lift
                SetLiftTarget(0);
                //wait to lift
                sleep(1000);
                //drive to the junction
                Move(0f,9f,0f,.3f);
                //wait a bit to be sure
                sleep(200);
                //drop cone
                OpenClaw();
                sleep(200);
                //move back off of junction
                Move(0f,-3f,0f,.3f);
                //lower
                SetLiftTarget(-1);
                Move(0,0f,-3.92f,.45f);
                //strafe into position
                Move(-.5f,0f,0f,.4f);

                Move(0,-22f,0,.45f);
                Move(20f,0,0,.45f);

                //endregion
                break;
            case 3:
                //pull off of wall
                Move(1.5f,0,0,.3f);
                //drive to the entrypoint
                Move(0,20,0,.45f);
                //Lift
                SetLiftTarget(2);
                //Drive to High junction
                Move(24f,0,0,.45f);
                //drive on top of high Junction
                Move(0,4,0,.45f);
                //drop cone
                OpenClaw();
                //move off of junction
                Move(0,-4,0,.45f);
                //lower lift
                SetLiftTarget(-1);
                CloseClaw();
                break;
            case 4:
                //pull off of wall
                Move(1.5f,0,0,.3f);
                //drive to the entrypoint
                Move(0,20,0,.45f);
                //Lift
                SetLiftTarget(2);
                //Drive to High junction
                Move(24f,0,0,.45f);
                //drive on top of high Junction
                Move(0,4,0,.45f);
                //drop cone
                OpenClaw();
                //move off of junction
                Move(0,-4,0,.45f);
                //lower lift
                SetLiftTarget(-1);
                CloseClaw();
                //drive to position 2
                Move(6,0,0,.45f);
                Move(0,-20,0,.45f);
                break;
            case 5:
                //pull off of wall
                Move(1.5f,0,0,.3f);
                //drive to the entrypoint
                Move(0,20,0,.45f);
                //Lift
                SetLiftTarget(2);
                //Drive to High junction
                Move(24f,0,0,.45f);
                //drive on top of high Junction
                Move(0,4,0,.45f);
                //drop cone
                OpenClaw();
                //move off of junction
                Move(0,-4,0,.45f);
                //lower lift
                SetLiftTarget(-1);
                CloseClaw();
                //drive to position 2
                Move(6,0,0,.45f);
                Move(0,-40,0,.45f);
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

    void  SetLiftTarget(int level){
        int targetRotations = 0;

        switch (level) {
            case -1:
                targetRotations = 0;
                break;
            case 0:
                targetRotations = (int) (538 * 3);
                break;
            case 1:
                targetRotations = (int) (538 * 4.5);
                break;
            case 2:
                targetRotations = 3340;
                break;
            default:
                break;
        }

        lift.setTargetPosition(targetRotations);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.65);
    }

    public void Move(float X, float Z,float thetaR, float speed){
        double totalTurnCircumference = 26.7f;
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
                t += (deltaLeft - deltaRight);
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
                t += (deltaLeft - deltaRight)/2;
                telemetry.addData("Turn: ", t);
                telemetry.update();
                driveTrainCode.UpdateDriveTrain(new Vector3(0, -speed, 0));
            }
        }

        //endregion
        driveTrainCode.UpdateDriveTrain(new Vector3(0,0,0));
    }

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
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("Label: ", String.valueOf(i)," ", recognition.getLabel());
                        CheckEndPoint(recognition.getLabel());
                        i++;
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

    void  SetStartPoint(){
        while(startPoint == StartPoint.UnSet && !isStopRequested())
        {
            telemetry.addData("A: ","Left");
            telemetry.addData("X: ","Right");
            //telemetry.addData("Y: ","Blue Side Left");
            //telemetry.addData("B: ","Blue Side Right");
            telemetry.update();
            if (gamepad1.a)
            {
                startPoint = StartPoint.redLeft;
                return;
            }
            if (gamepad1.x)
            {
                startPoint = StartPoint.redRight;
                return;
            }
            /*
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

    private void OpenClaw(){
        grabber.setPosition(.22f);
    }

    private void CloseClaw(){
        grabber.setPosition(.35f);
    }

    private void InitVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void InitTfod() {
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

    private void Initialize(){
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


        telemetry.addData("Path Set ", "Start: ", startPoint.name());
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

    }


    private void CheckForModel(){
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

}