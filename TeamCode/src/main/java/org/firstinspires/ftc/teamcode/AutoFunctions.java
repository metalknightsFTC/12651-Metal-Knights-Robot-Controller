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
    private Blinker Expansion_Hub_1;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotorEx right;
    private DcMotorEx left;
    private DcMotorEx rear;
    OdometryControl odometryControl;

    public void runOpMode() {

        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");


        right = hardwareMap.get(DcMotorEx.class, "frontRight");
        left = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rear = hardwareMap.get(DcMotorEx.class, "backRight");
        odometryControl = new OdometryControl(right, left, rear, hardwareMap, gamepad1);
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            while (odometryControl.robotPosition.z<940f){
                odometryControl.CalculatePosition();

                odometryControl.SetStickPower(0f,.5f,0f);

            }
            odometryControl.SetStickPower(0f,0,0f);
            odometryControl.CalculatePosition();
            telemetry.addData("x", odometryControl.robotPosition.x);
            telemetry.addData("z", odometryControl.robotPosition.z);
            telemetry.addData("turn", odometryControl.robotPosition.currentHeading);
            telemetry.update();

        }
    }
}