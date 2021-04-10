package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ServoTest", group = "TeleOp")

public class ServoTest extends LinearOpMode {
    //All hardware
    protected DcMotor motorFwdLeft = null;
    protected DcMotor motorFwdRight = null;
    protected DcMotor motorBackLeft = null;
    protected DcMotor motorBackRight = null;
    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;
    public DcMotor intakeFwd = null;
    public DcMotor intakeBack = null;
    public Servo basketServo = null;
    public Servo unloadServo = null;
    public Servo wobbleGrabberServo = null;
    public Servo wobbleArmServo = null;
    public Servo ringPullPivotServo = null;
    public Servo ringPullArmServo = null;
    public Rev2mDistanceSensor distanceFwdLeft = null;
    public Rev2mDistanceSensor distanceFwdRight = null;
    public Rev2mDistanceSensor distanceLeft = null;


    double servo = 0.5;
    double servo2 = 0.5;
@Override
    public void runOpMode() {
        //Prepares all the hardware
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        intakeFwd = hardwareMap.get(DcMotor.class, "intakeFwd");
        intakeBack = hardwareMap.get(DcMotor.class, "intakeBack");

        basketServo = hardwareMap.get(Servo.class,"basketServo");
        unloadServo = hardwareMap.get(Servo.class,"unloadServo");
        wobbleGrabberServo = hardwareMap.get(Servo.class, "wobbleGrabberServo");
        wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");
        ringPullPivotServo = hardwareMap.get(Servo.class, "ringPullPivotServo");
        ringPullArmServo = hardwareMap.get(Servo.class, "ringPullArmServo");

        distanceFwdLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdLeft");
        distanceFwdRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdRight");
        distanceLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

       // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                servo += 0.001;
            }
            else if(gamepad1.dpad_down){
                servo -= 0.001;
            }
            /*if(gamepad1.dpad_left){
                servo2 += 0.001;
            }
            else if(gamepad1.dpad_right){
                servo2 -= 0.001;
            }

             */
            unloadServo.setPosition(servo);
            //ringPullPivotServo.setPosition(servo2);

            //telemetry.addData("unloadServo",wobbleArmServo.getPosition());
            telemetry.addData("servo", servo);
           // telemetry.addData("servo2", servo2);
            telemetry.addData("Distance  Left", distanceLeft.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Forward Right", distanceFwdRight.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Forward Left", distanceFwdLeft.getDistance(DistanceUnit.MM));

            telemetry.update();
        }
    }
}
