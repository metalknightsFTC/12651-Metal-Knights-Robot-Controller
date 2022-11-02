package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

    DriveTrainCode driveTrainCode;

    private int targetRotations = 0;
    float currentSpeed = .6f;
    float slowSpeed = .3f;
    float regularSpeed = .6f;

    @Override
    public void runOpMode(){
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        imu = new IMUController(hardwareMap);
        lift = hardwareMap.get(DcMotor.class,"lifter");
        grabber = hardwareMap.get(Servo.class,"grabber");

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
                driveTrainCode.UpdateDriveTrain(new Vector3(-currentSpeed,StrafeCorrection(),0));
            }else if(gamepad1.dpad_left) {
                driveTrainCode.UpdateDriveTrain(new Vector3(currentSpeed,StrafeCorrection(),0));
            }
            else {
                if(driveTrainCode.RSX >= 0.001 || driveTrainCode.RSX <= -0.001 || !(driveTrainCode.LSX > 0.02f || driveTrainCode.LSX < -0.02f))
                {
                    imu.ResetAngle();
                }
                if(driveTrainCode.LSX > 0.02f || driveTrainCode.LSX < 0.02f)
                {
                    driveTrainCode.UpdateDriveTrain(currentSpeed, StrafeCorrection());
                }else
                {
                    driveTrainCode.UpdateDriveTrain(currentSpeed, StrafeCorrection());
                }
            }

            //region lifter buttons
            //538
            if(gamepad1.a){
                targetRotations = (538 * 4);
            }
            if(gamepad1.x){
                targetRotations = (int)(538 * 6.5);
            }
            if(gamepad1.y){
                targetRotations = 4650;
            }
            if(gamepad1.b){
                targetRotations = 0;
            }
            if(gamepad1.dpad_up){
                targetRotations += (538);
            }
            if(gamepad1.dpad_down){
                targetRotations -= (538);
            }
            //endregion

            //region lifter code
            targetRotations += (gamepad1.right_trigger - gamepad1.left_trigger) * 10;
            if(targetRotations < 0){
                targetRotations = 0;
            }
            if(targetRotations > 4650){
                targetRotations = 4650;
            }
            lift.setTargetPosition(targetRotations);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);

            if(gamepad1.left_bumper){
                grabber.setPosition(.22f);
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

    public  void LockToHeading()
    {

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

}