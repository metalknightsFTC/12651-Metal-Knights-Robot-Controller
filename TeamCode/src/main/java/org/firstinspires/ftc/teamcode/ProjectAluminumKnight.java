package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Vectors.*;

@TeleOp
public class ProjectAluminumKnight extends LinearOpMode {

    Blinker Expansion_Hub_1;
    Blinker Expansion_Hub_2;
    Servo grabber;
    private IMUController imu;
    DriveTrainCode driveTrainCode;

    LiftManager lift;

    public static float currentSpeed = .6f;
    public static float slowSpeed = .3f;
    public static float regularSpeed = .6f;
    public static float fastSpeed = .8f;
    public static TouchSensor poleContact;

    Servo verticalR;
    Servo horizontalR;

    boolean liftLimits = true;

    @Override
    public void runOpMode()
    {
        Initialize();
        waitForStart();
        RunLoop();
    }

    public void RunLoop()
    {
        while (opModeIsActive())
        {
            //verticalR.setPosition(.27);
            //verticalR.setPosition(.655);
            //horizontalR.setPosition(.04);
            LiftRecovery();

            SpeedAndDrive();
            ManageLiftLevel();
            //Contact();
            //region Grabber Code
            if(gamepad1.left_bumper){
                grabber.setPosition(0.13f);
            }else{
                grabber.setPosition(.33f);
            }
            //endregion

            imu.GetAngle();
            telemetry.addData("Angle : ", imu.heading);
            telemetry.update();
            //endregion
        }
    }
    public void SpeedAndDrive()
    {
        if(gamepad1.right_bumper)
        {
            currentSpeed = slowSpeed;
        }else if(gamepad1.left_stick_button)
        {
            currentSpeed = fastSpeed;
        }else
        {
            currentSpeed = regularSpeed;
        }
        if(driveTrainCode.RSX >= 0.001 || driveTrainCode.RSX <= -0.001 || !(driveTrainCode.LSX > 0.02f || driveTrainCode.LSX < -0.02f))
        {
            imu.ResetAngle();
        }
        driveTrainCode.UpdateDriveTrain(currentSpeed, StrafeCorrection());
    }

    public void ManageLiftLevel()
    {
        //region lifter buttons
        //538

        if(gamepad1.dpad_right){
            lift.Lift(4);
        }
        if(gamepad1.dpad_left) {
            lift.Lift(2);
        }
        if(gamepad1.dpad_up){
            lift.Lift(2);
        }
        if(gamepad1.dpad_down){
            lift.Lift(1);
        }
        if(gamepad1.a){
            lift.Lift(5);
        }
        if(gamepad1.x){
            lift.Lift(6);
        }
        if(gamepad1.y){
            lift.Lift(7);
        }
        if(gamepad1.b){
            lift.Lift(0);
        }
        //endregion

        //region lifter code
        lift.Lift((gamepad1.right_trigger - gamepad1.left_trigger)* 20f, liftLimits);

    }

    public void Contact()
    {
        if(poleContact.isPressed()){
            gamepad1.rumble(.5,.5,100);
        }else{
            gamepad1.stopRumble();
        }
    }

    public void Initialize(){
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        imu = new IMUController(hardwareMap);
        lift = new LiftManager(hardwareMap);
        horizontalR = hardwareMap.get(Servo.class,"alignment");
        verticalR = hardwareMap.get(Servo.class,"pivot");
        grabber = hardwareMap.get(Servo.class,"grabber");
        //poleContact = hardwareMap.get(TouchSensor.class, "poleContact");
        driveTrainCode = new DriveTrainCode(gamepad1,hardwareMap);

        driveTrainCode.InvertMotorDirection(Motor.backLeft);
        driveTrainCode.InvertMotorDirection(Motor.frontLeft);
        imu.ResetAngle();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public float StrafeCorrection()
    {
        float turnMod = (float) imu.AngleDeviation(0) / 20;
        turnMod = Range(turnMod,-1,1);
        return -turnMod;
    }

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

    public void LiftRecovery()
    {
        telemetry.addData("Recover Lift: ","Hold A to unlock");
        liftLimits = !gamepad2.a;
        if(gamepad2.b && gamepad2.a){
            lift.Reset();
        }
    }

}