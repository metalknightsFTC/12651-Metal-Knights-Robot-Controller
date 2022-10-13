package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.SelectedDrive;

@TeleOp
public class ProjectAluminumKnight extends LinearOpMode {

    Blinker Expansion_Hub_1;
    Blinker Expansion_Hub_2;
    DcMotor lift;
    Servo grabber;

    DcMotorEx right;
    DcMotorEx left;
    DcMotorEx rear;
    private boolean automatics = false;
    private int targetRotations = 0;
    OdometryControl odometryControl;

    float speed = .6f;

    @Override
    public void runOpMode(){
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");

        lift = hardwareMap.get(DcMotor.class,"lifter");
        grabber = hardwareMap.get(Servo.class,"grabber");
        right = hardwareMap.get(DcMotorEx.class,"right");
        left = hardwareMap.get(DcMotorEx.class,"left");
        rear = hardwareMap.get(DcMotorEx.class,"rear");

        DriveTrainCode driveTrainCode = new DriveTrainCode(gamepad1,hardwareMap);

        driveTrainCode.InvertMotorDirection(Motor.backLeft);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        odometryControl = new OdometryControl(right,left,rear,new Vector3(0,0,0));
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            if(gamepad1.dpad_up){
                driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,-.5f));
            }else if(gamepad1.dpad_down){
                driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(0,0,.5f));
            }else if(gamepad1.dpad_right){
                driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(-.5f,0,0));
            }else if(gamepad1.dpad_left) {
                driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,new Vector3(.5f,0,0));
            }
            else {
                driveTrainCode.UpdateDriveTrain(SelectedDrive.mecanum, speed);
            }

            //region lifter buttons
            //538
            if(gamepad1.a){
                targetRotations = (int)(538 * 3);
            }
            if(gamepad1.x){
                targetRotations = (int)(538 * 4.5);
            }
            if(gamepad1.y){
                targetRotations = 3330;
            }
            if(gamepad1.b){
                targetRotations = 0;
            }
            //endregion

            //region lifter code
            targetRotations += (gamepad1.right_trigger - gamepad1.left_trigger) * 5;
            if(targetRotations < 0){
                targetRotations = 0;
            }
            if(targetRotations > 3330){
                targetRotations = 3330;
            }
            lift.setTargetPosition(targetRotations);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(.75);

            if(gamepad1.left_bumper){
                grabber.setPosition(.22f);
            }else{
                grabber.setPosition(.35f);
            }

            telemetry.addData("Lift Target: ", ((double)targetRotations));
            telemetry.addData("Current Lift Position: ", lift.getCurrentPosition());
            telemetry.update();
            //endregion
        }

    }
}