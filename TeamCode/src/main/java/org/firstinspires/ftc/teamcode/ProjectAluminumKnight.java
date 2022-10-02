package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
        odometryControl = new OdometryControl(right,left,rear,new Vector3(0,0,0));
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            driveTrainCode.UpdateDriveTrain(SelectedDrive.mecanum,.5f);
            UpdatePosition();

            if(gamepad1.back && !automatics){
                automatics = true;
            }else if(gamepad1.back){
                automatics = false;
                targetRotations = 0;
            }
            if(gamepad1.left_bumper){
                grabber.setPosition(0);
            }else{
                grabber.setPosition(.35);
            }

            //region lifter buttons
            //538
            if(gamepad1.a){
                targetRotations = (int)(538 * 1.7);
            }
            if(gamepad1.x){
                targetRotations = (int)(538 * 4.3);
            }
            if(gamepad1.y){
                targetRotations = 3340;
            }
            if(gamepad1.b){
                targetRotations = 0;
            }
            //endregion

            telemetry.addData("AP = ", automatics);

            //region lifter code
            if(automatics){
                //double liftDelta = (targetRotations - (lift.getCurrentPosition()) * .1);
                lift.setTargetPosition(targetRotations);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //lift.setPower(liftDelta);
                lift.setPower(.75);
            }else{

                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(gamepad1.dpad_up)
                    lift.setPower(.75);
                else if(gamepad1.dpad_down)
                    lift.setPower(-.75);
                else
                    lift.setPower(0);
            }

            telemetry.addData("Lift Target: ", ((double)targetRotations));
            telemetry.addData("Current Lift Position: ", lift.getCurrentPosition());
            telemetry.update();
            //endregion
        }

    }

    void  UpdatePosition()
    {
        double[] deltas = odometryControl.CalculateRobotPosition();

        telemetry.addData("Right: ",right.getCurrentPosition());
        telemetry.addData("Left",left.getCurrentPosition());
        telemetry.addData("rear", rear.getCurrentPosition());

        odometryControl.robotPosition.currentHeading += deltas[2];
        odometryControl.robotPosition.x += deltas[0];
        odometryControl.robotPosition.z += deltas[1];
        telemetry.addData("X: ",odometryControl.robotPosition.x);
        telemetry.addData("Z: ",odometryControl.robotPosition.z);
        telemetry.addData("Heading: ",odometryControl.robotPosition.currentHeading);
    }

}