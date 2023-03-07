package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.Enums.EndPoint;
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.StartPoint;
import org.firstinspires.ftc.teamcode.UtilityClasses.DriveTrainController;
import org.firstinspires.ftc.teamcode.UtilityClasses.IMUController;
import org.firstinspires.ftc.teamcode.UtilityClasses.LiftManager;
import org.firstinspires.ftc.teamcode.UtilityClasses.NavigationManager;
import org.firstinspires.ftc.teamcode.Vectors.Vector2;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class Training extends LinearOpMode {

    private Servo grabber;
    private Servo flipOut;
    private DriveTrainController driveTrainController;
    private NavigationManager navSystem;
    Servo verticalR;
    Servo horizontalR;
    private LiftManager liftManager;
    private IMUController imu;

    @Override
    public void runOpMode()
    {
        liftManager = new LiftManager(hardwareMap);
        imu = new IMUController(hardwareMap);
        while(!imu.IMU_Calibrated()){
            telemetry.addData("IMU STATUS: ","Calibrating");
            telemetry.update();
        }
        Initialize();
        waitForStart();



    }

    //region Servo Controls
    private void SetCameraAngle(double x, double z){
        verticalR.setPosition(z);
        horizontalR.setPosition(x);
    }

    private void OpenClaw(){
        grabber.setPosition(.16f);
    }

    private void CloseClaw(){
        grabber.setPosition(.33f);
    }
    //endregion

    private void Initialize()
    {
        Blinker expansion_Hub_1 = imu.Expansion_Hub_1;
        Blinker expansion_Hub_2 = imu.Expansion_Hub_2;

        horizontalR = hardwareMap.get(Servo.class,"alignment");
        verticalR = hardwareMap.get(Servo.class,"pivot");
        grabber = hardwareMap.get(Servo.class,"grabber");
        flipOut = hardwareMap.get(Servo.class,"flipOut");

        driveTrainController = new DriveTrainController(gamepad1 ,hardwareMap);

        driveTrainController.InvertMotorDirection(Motor.backRight);
        driveTrainController.InvertMotorDirection(Motor.frontRight);

        navSystem = new NavigationManager(hardwareMap, imu, driveTrainController);
        telemetry.addData("IMU STATUS: ","Calibrated");
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

    }
}