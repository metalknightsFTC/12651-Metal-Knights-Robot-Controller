package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Vectors.*;

@TeleOp
public class ProjectAluminumKnight extends LinearOpMode {

    Blinker Expansion_Hub_1;
    Blinker Expansion_Hub_2;
    DcMotor lift;
    Servo grabber;

    DriveTrainCode driveTrainCode;

    DcMotorEx right;
    DcMotorEx left;
    DcMotorEx rear;
    private int targetRotations = 0;
    float currentSpeed = .6f;
    float slowSpeed = .3f;
    float regularSpeed = .6f;

    int cpr = 8192;
    double c = 6.1575216;//(Math.PI) * (diameter / 2);//6.1575216

    double currentLeftEncoderRotation = 0;
    double currentRightEncoderRotation = 0;
    double currentRearEncoderRotation = 0;
    double leftEncoderRotation = 0;
    double rightEncoderRotation = 0;
    double rearEncoderRotation = 0;

    @Override
    public void runOpMode(){
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");

        lift = hardwareMap.get(DcMotor.class,"lifter");
        grabber = hardwareMap.get(Servo.class,"grabber");
        right = hardwareMap.get(DcMotorEx.class,"right");
        left = hardwareMap.get(DcMotorEx.class,"left");
        rear = hardwareMap.get(DcMotorEx.class,"rear");

        driveTrainCode = new DriveTrainCode(gamepad1,hardwareMap);

        driveTrainCode.InvertMotorDirection(Motor.backLeft);
        driveTrainCode.InvertMotorDirection(Motor.frontLeft);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            if(gamepad1.dpad_up){
                driveTrainCode.UpdateDriveTrain(new Vector3(0,0,-.5f));
            }else if(gamepad1.dpad_down){
                driveTrainCode.UpdateDriveTrain(new Vector3(0,0,.5f));
            }else if(gamepad1.dpad_right){
                driveTrainCode.UpdateDriveTrain(new Vector3(-.5f,0,0));
            }else if(gamepad1.dpad_left) {
                driveTrainCode.UpdateDriveTrain(new Vector3(.5f,0,0));
            }
            else {
                if(gamepad1.right_bumper){
                    currentSpeed = slowSpeed;
                }else{
                    currentSpeed = regularSpeed;
                }
                driveTrainCode.UpdateDriveTrain(currentSpeed,0);

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
            //endregion

            //region lifter code
            targetRotations += (gamepad1.right_trigger - gamepad1.left_trigger) * 5;
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

            //RegulateMotors();

            telemetry.addData("Lift Target: ", ((double)targetRotations));
            telemetry.addData("Current Lift Position: ", lift.getCurrentPosition());
            telemetry.update();
            //endregion
        }

    }

    double totalMovementX = 0;
    double totalMovementZ = 0;
    double t = 0;

    public  float StrafeCorrection(){
        double deltaLeft;
        double deltaRight;
        double deltaBack;

        //region X checks
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

        float turnMod = (float) ((deltaLeft-deltaRight)/2.5);

        return -turnMod;

    }

}