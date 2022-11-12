package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    DcMotor lift;
    Servo grabber;
    private IMUController imu;
    Orientation angles;
    Acceleration gravity;
    private DcMotorEx right;
    private DcMotorEx left;
    private DcMotorEx rear;
    DriveTrainCode driveTrainCode;

    private int targetRotations = 0;
    public static float currentSpeed = .6f;
    public static float slowSpeed = .3f;
    public static float regularSpeed = .6f;
    public static int tpr = 8192;//found on REV encoder specs chart
    public static double c = 6.1575216; //Circumference of dead wheels (Math.PI) * (diameter / 2);//6.1575216

    public static float rampDown = 4.5f;

    public  static float coordinateX;
    public  static float coordinateZ;

    @Override
    public void runOpMode(){
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        imu = new IMUController(hardwareMap);

        lift = hardwareMap.get(DcMotor.class,"lifter");
        grabber = hardwareMap.get(Servo.class,"grabber");
        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");
        rear = hardwareMap.get(DcMotorEx.class, "rear");

        driveTrainCode = new DriveTrainCode(gamepad1,hardwareMap);

        driveTrainCode.InvertMotorDirection(Motor.backLeft);
        driveTrainCode.InvertMotorDirection(Motor.frontLeft);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.ResetAngle();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            if(gamepad1.right_bumper){
                currentSpeed = slowSpeed;
            }else{
                currentSpeed = regularSpeed;
            }

            if(gamepad1.dpad_right){
                LockToHeading(45);
            }else if(gamepad1.dpad_left) {
                LockToHeading(-45);
            }
            else {
                if(driveTrainCode.RSX >= 0.001 || driveTrainCode.RSX <= -0.001 || !(driveTrainCode.LSX > 0.02f || driveTrainCode.LSX < -0.02f))
                {
                    imu.ResetAngle();
                }
                driveTrainCode.UpdateDriveTrain(currentSpeed, StrafeCorrection());
            }

            //region lifter buttons
            //538
            if(gamepad1.a){
                targetRotations = (1776);
            }
            if(gamepad1.x){
                targetRotations = (2906);
            }
            if(gamepad1.y){
                targetRotations = 4150;
            }
            if(gamepad1.b){
                targetRotations = 0;
            }
            if(gamepad1.dpad_up){
                targetRotations += (int)((538) * 1.5);
            }
            if(gamepad1.dpad_down){
                targetRotations -= (int)((538) * 1.5);
            }
            //endregion

            //region lifter code
            targetRotations += (gamepad1.right_trigger - gamepad1.left_trigger) * 20;
            if(targetRotations < 0){
                targetRotations = 0;
            }
            if(targetRotations > 4200){
                targetRotations = 4200;
            }
            lift.setTargetPosition(targetRotations);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);

            if(gamepad1.left_bumper){
                grabber.setPosition(0.13f);
            }else{
                grabber.setPosition(.33f);
            }

            imu.GetAngle();
            telemetry.addData("Angle : ", imu.heading);
            telemetry.addData("Lift Target: ", ((double)targetRotations));
            telemetry.addData("Current Lift Position: ", lift.getCurrentPosition());
            telemetry.update();
            //endregion
        }

    }

    public  void LockToHeading(float target)
    {
        float deltaNC = (float) (target - imu.GetAngle());
        float rampDown = 3;

        if(currentSpeed > .6f)
        {
            rampDown = 18;
        }

        if (deltaNC > .04f || deltaNC < -.04f)
        {
            float delta = (float) (target - imu.GetAngle()) / rampDown;

            driveTrainCode.UpdateDriveTrain(new Vector3(0, currentSpeed * delta, 0));
            telemetry.addData("CurrentHeading: ", imu.GetAngle());
            telemetry.update();
        }
        driveTrainCode.UpdateDriveTrain(new Vector3(0,0,0));
        imu.ResetAngle();
    }

    public  float StrafeCorrection()
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

    public void Odometry()
    {
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

        leftEncoderRotation = currentLeftEncoderRotation;
        rightEncoderRotation = currentRightEncoderRotation;
        rearEncoderRotation = currentRearEncoderRotation;

        currentRightEncoderRotation = right.getCurrentPosition();
        currentRearEncoderRotation = rear.getCurrentPosition();
        currentLeftEncoderRotation = left.getCurrentPosition();

        deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / tpr) * c;
        deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / tpr) * c;
        deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / tpr) * c;
        totalMovementZ += (deltaLeft + deltaRight) / 2;
        totalMovementX += deltaBack;

        coordinateX += totalMovementX;
        coordinateZ += totalMovementZ;
        telemetry.addData("X position: ", coordinateX);
        telemetry.addData("Z position: ", coordinateZ);
    }

}