package org.firstinspires.ftc.teamcode.Templates;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.UtilityClasses.DriveTrainController;
import org.firstinspires.ftc.teamcode.Enums.EndPoint;
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.StartPoint;
import org.firstinspires.ftc.teamcode.UtilityClasses.IMUController;
import org.firstinspires.ftc.teamcode.UtilityClasses.LiftManager;
import org.firstinspires.ftc.teamcode.UtilityClasses.NavigationManager;
import org.firstinspires.ftc.teamcode.Vectors.Vector2;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class Autonomous_Template extends LinearOpMode {

    private EndPoint endPoint = EndPoint.unset;
    private StartPoint startPoint = StartPoint.UnSet;
    private DriveTrainController driveTrainController;
    private NavigationManager navSystem;
    private int routine = 0;
    public static double targetHeading = 0;
    private IMUController imu;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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
        imu = new IMUController(hardwareMap);
        while(!imu.IMU_Calibrated()){
            telemetry.addData("IMU STATUS: ","Calibrating");
            telemetry.update();
        }
        Initialize();
        waitForStart();
        imu.ResetAngle();
        targetHeading = imu.GetAngle();
        SetRoutine();
        RunRoutine();
    }

    //region Run the Autonomous
    private void  RunRoutine() {
        switch (routine) {
            case 0:
                //region Red Left Case 0
                RedLeftSide();

                //endregion
                break;

            case 1:
                //region Red Left Case 1
                RedLeftSide();

                //endregion
                break;
            case 2:
                //region Red Left Case 2
                RedLeftSide();

                //endregion
                break;
            case 3:
                //region Red Right Case 3
                RedRightSide();
                sleep(3200);
                //endregion
                break;
            case 4:
                //region Red Right Case 4
                RedRightSide();
                //endregion
                break;
            case 5:
                //region Red Right Case 5
                BlueLeftSide();
                //endregion
                break;
            case 6:
                BlueLeftSide();
                break;
            case 7:
                BlueLeftSide();
                break;
            case 8:
                BlueLeftSide();
                break;
            case 9:
                BlueRightSide();
                break;
            case 10:
                BlueRightSide();
                break;
            case 11:
                BlueRightSide();
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
    //endregion

    //endregion

    private void Initialize()
    {
        Blinker expansion_Hub_1 = imu.Expansion_Hub_1;
        Blinker expansion_Hub_2 = imu.Expansion_Hub_2;

        driveTrainController = new DriveTrainController(gamepad1 ,hardwareMap);

        driveTrainController.InvertMotorDirection(Motor.backRight);
        driveTrainController.InvertMotorDirection(Motor.frontRight);

        navSystem = new NavigationManager(hardwareMap, imu, driveTrainController);
        SetStartPoint();

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

    private void RedLeftSide()
    {

    }



    private void RedRightSide()
    {

    }

    private void BlueLeftSide()
    {

    }


    private void BlueRightSide()
    {

    }
}