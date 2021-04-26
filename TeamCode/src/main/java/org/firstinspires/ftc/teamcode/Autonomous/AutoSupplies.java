package org.firstinspires.ftc.teamcode.Autonomous;

//imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.AutoSupplies;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

//import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;


public abstract class AutoSupplies extends LinearOpMode{
    //  Establish hardware
    //motors
    protected DcMotor  motorFwdLeft   = null;
    protected DcMotor  motorFwdRight  = null;
    protected DcMotor  motorBackLeft  = null;
    protected DcMotor  motorBackRight = null;
    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;
    public DcMotor intakeFwd = null;
    public DcMotor intakeBack = null;
    public RevBlinkinLedDriver lights;
    protected BNO055IMU imu;
    //sensors
    protected Rev2mDistanceSensor distanceFwdLeft = null;
    protected Rev2mDistanceSensor distanceFwdRight = null;
    protected Rev2mDistanceSensor distanceBackLeft = null;
    protected Rev2mDistanceSensor distanceBackRight = null;
    protected Rev2mDistanceSensor distanceLeftTop = null;
    protected Rev2mDistanceSensor distanceLeftBottom = null;

    //servos
    public Servo basketServo = null;
    public Servo unloadServo = null;
    public Servo wobbleArmServo = null;
    public Servo wobbleGrabberServo = null;
    public Servo ringPullPivotServo = null;
    public Servo ringPullArmServo = null;
    //camera
    public AutoSupplies.SkystoneDeterminationPipeline pipeline;

    //  Declare OpMode Members
    protected ElapsedTime runtime = new ElapsedTime();
    abstract public void runOpMode() throws InterruptedException;

    //  Protected variables
    protected double globalAngle;
    protected double globalPitch;
    protected Orientation lastAngles = new Orientation();
    protected Orientation lastPitches = new Orientation();

    //Encoder Values
    //Neverest 40 motor spec: quadrature encoder, 7 pulses per revolution, count = 7 * 40
    private static final double COUNTS_PER_MOTOR_REV = 280; // Neverest 40 motor encoder
    private static final double DRIVE_GEAR_REDUCTION = 1; // This is < 1 if geared up
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;
    //---callable methods---

    //move
    public void move(long millis, double x, double y)
    {
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;
        double maxPower;
        double max = 1.0;

        maxPower = Math.abs(leftFrontPower);
        if (Math.abs(rightFrontPower) > maxPower) {
            maxPower = Math.abs(rightFrontPower);
        }
        if (Math.abs(leftBackPower) > maxPower) {
            maxPower = Math.abs(leftBackPower);
        }
        if (Math.abs(rightBackPower) > maxPower) {
            maxPower = Math.abs(rightBackPower);
        }
        if (maxPower > 1) {
            leftFrontPower = leftFrontPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightBackPower = rightBackPower / maxPower;

        }
        //sets the power of the motors
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis) {
            motorFwdLeft.setPower(leftFrontPower*max);
            motorFwdRight.setPower(rightFrontPower*max);
            motorBackLeft.setPower(leftBackPower*max);
            motorBackRight.setPower(rightBackPower*max);
        }
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    public void encoderMove(double degrees, double x, double y){
        resetDriveEncoders();
        double counts = degrees * COUNTS_PER_DEGREE1;
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;
        double maxPower;
        double max = 1.0;
        double posPower = 0.2;
        maxPower = Math.abs(leftFrontPower);
        if (Math.abs(rightFrontPower) > maxPower) {
            maxPower = Math.abs(rightFrontPower);
        }
        if (Math.abs(leftBackPower) > maxPower) {
            maxPower = Math.abs(leftBackPower);
        }
        if (Math.abs(rightBackPower) > maxPower) {
            maxPower = Math.abs(rightBackPower);
        }
        if (maxPower > 1) {
            leftFrontPower = leftFrontPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightBackPower = rightBackPower / maxPower;

        }
        //sets the power of the motors
        double averageEnc = (Math.abs(motorFwdLeft.getCurrentPosition())
                + Math.abs(motorFwdRight.getCurrentPosition())
                + Math.abs(motorBackLeft.getCurrentPosition())
                + Math.abs(motorBackRight.getCurrentPosition()))/4.0;
        while (opModeIsActive() && averageEnc <= counts){
            averageEnc = (Math.abs(motorFwdLeft.getCurrentPosition())
                    + Math.abs(motorFwdRight.getCurrentPosition())
                    + Math.abs(motorBackLeft.getCurrentPosition())
                    + Math.abs(motorBackRight.getCurrentPosition()))/4.0;
            if(posPower < 1 && averageEnc/counts < .6){
                posPower *= 1.1;
            }
            else if(posPower >= 1 && averageEnc/counts <.6){
                posPower = 1;
            }
            else if(averageEnc/counts >= .6 && posPower >= .25){
                posPower *= .99;
            }
            else{
                posPower = .25;
            }
            motorFwdLeft.setPower(leftFrontPower*max*posPower);
            motorBackLeft.setPower(leftBackPower*max*posPower);
            motorFwdRight.setPower(rightFrontPower*max*posPower);
            motorBackRight.setPower(rightBackPower*max*posPower);
        }
        motorFwdLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackRight.setPower(0);
    }
    public void resetDriveEncoders(){
        motorFwdLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFwdRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFwdRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setPower(double x, double y)
    {
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;

        motorFwdLeft.setPower(leftFrontPower);
        motorFwdRight.setPower(rightFrontPower);
        motorBackLeft.setPower(leftBackPower);
        motorBackRight.setPower(rightBackPower);
    }
    public void turn(int degrees, double power){
        int left = 1;
        int right = 1;
        resetAngle();
        telemetry.addData("Angle",getAngle());
        telemetry.update();
        if(degrees >= 0){
            right *= -1;
        }
        else if(degrees < 0){
            left *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power* right);

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}

        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
    }
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnToS(int degrees, double power, int loopnum){
        int left = 1;
        int right = 1;
        double distance = getAngle() - degrees;
        double startAngle = getAngle();
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() <= degrees){
            left *= -1;
        }
        else if(getAngle() > degrees){
            right *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                //telemetry.addData("Angle4",getAngle());
                //telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                //telemetry.addData("Angle4", getAngle());
                //telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
        if(--loopnum > 0){
            turnToS(degrees, power/2, loopnum);
        }
    }
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnTo(int degrees, double power){
        int left = 1;
        int right = 1;
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() >= degrees){
            left *= -1;
        }
        else if(getAngle() < degrees){
            right *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                //telemetry.addData("Angle4",getAngle());
                //telemetry.update();
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                //telemetry.addData("Angle4", getAngle());
                //telemetry.update();
            }
        }
        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
    }
    public void alignDistanceFwd(){

    }
    //  Pause for the specified amount of time (time: mili secs)
    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }
    //Resets gyro sensor bearing value to 0
    //commonly used to calibrate before a match as well
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    //Resets gyro sensor pitch value to 0
    //commonly used to calibrate before a match as well
    public void resetPitch()
    {
        lastPitches = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalPitch = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */


    //uses the imu to find the current angle
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    //uses the imu to get the current pitch of the robot
    public double getPitch() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.secondAngle - lastPitches.secondAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalPitch += deltaAngle;

        lastPitches = angles;

        return globalPitch;
    }
    public double getDistanceFwdLeft(){
        return distanceFwdLeft.getDistance(DistanceUnit.MM);
    }
    public double getDistanceFwdRight(){
        return distanceFwdRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceBackLeft(){
        return distanceBackRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceBackRight(){
        return distanceBackRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceLeftTop(){ return distanceLeftTop.getDistance(DistanceUnit.MM); }
    public double getDistanceLeftBottom(){ return distanceLeftBottom.getDistance(DistanceUnit.MM); }

    public void moveUsingLeftDistance(double distance, double power){//- means strafe left and + means strafe right
        while (opModeIsActive()) {
            double dist = distanceLeftTop.getDistance(DistanceUnit.MM);
            double dist2 = distanceLeftBottom.getDistance(DistanceUnit.MM);
            //time out check
            if(dist == 65535 || distanceLeftTop.didTimeoutOccur()) {
                dist = 65535;
            }
            if(dist2 == 65535 || distanceLeftBottom.didTimeoutOccur()){
                dist2 = 65535;
            }
            //telemetry
            telemetry.addData("distance val 1", dist);
            telemetry.addData("distance val 2", dist2);
            //power set or loop break if distance traveled is met
            if(power <= 0) {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 > distance) {
                    setPower(power, 0);
                } else if (dist != 65535 && dist2 == 65535 && dist > distance) {
                    setPower(power, 0);
                } else if (dist == 65535 && dist2 != 65535 && dist2 > distance) {
                    setPower(power, 0);
                } else {
                    break;
                }
            }
            else {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 < distance) {
                    setPower(power, 0);
                } else if (dist != 65535 && dist2 == 65535 && dist < distance) {
                    setPower(power, 0);
                } else if (dist == 65535 && dist2 != 65535 && dist2 < distance) {
                    setPower(power, 0);
                } else {
                    break;
                }
            }
            telemetry.update();
        }
    }
    public void moveUsingFwdDistance(double distance, double power){//- means strafe back and + means strafe fwd
        while (opModeIsActive()) {
            double dist = distanceFwdLeft.getDistance(DistanceUnit.MM);
            double dist2 = distanceFwdRight.getDistance(DistanceUnit.MM);
            //time out check
            if(dist == 65535 || distanceFwdLeft.didTimeoutOccur()) {
                dist = 65535;
            }
            if(dist2 == 65535 || distanceFwdRight.didTimeoutOccur()){
                dist2 = 65535;
            }
            //telemetry
            telemetry.addData("distance val 1", dist);
            telemetry.addData("distance val 2", dist2);
            //power set or loop break if distance traveled is met
            if(power <= 0) {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 < distance) {
                    setPower(0, power);
                } else if (dist != 65535 && dist2 == 65535 && dist < distance) {
                    setPower(0, power);
                } else if (dist == 65535 && dist2 != 65535 && dist2 < distance) {
                    setPower(0, power);
                } else {
                    break;
                }
            }
            else {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 > distance) {
                    setPower(0, power);
                } else if (dist != 65535 && dist2 == 65535 && dist > distance) {
                    setPower(0, power);
                } else if (dist == 65535 && dist2 != 65535 && dist2 > distance) {
                    setPower(0, power);
                } else {
                    break;
                }
            }
            telemetry.update();
        }
    }
    //add functions to set positions of servos here
    public void basketServoDown(){
        basketServo.setPosition(0.641);
    }
    public void basketServoUp(){
        basketServo.setPosition(0.406999);
    }
    public void wobbleArmStart(){
        wobbleArmServo.setPosition(0.37399);
    }
    public void wobbleArmUp(){
        wobbleArmServo.setPosition(0.2830);
    }
    public void wobbleArmDown(){
        wobbleArmServo.setPosition(0.00999);
    }
    public void wobbleGrabberClosed(){
        wobbleGrabberServo.setPosition(0);
    }
    public void wobbleGrabberOpen(){
        wobbleGrabberServo.setPosition(0.319);
    }
    public void wobbleGrabberEject(){wobbleGrabberServo.setPosition(.921);}
    public void unloadServoPush(){ unloadServo.setPosition(.3239); }
    public void unloadServoBack(){ unloadServo.setPosition(.71); }
    public void unloadServoStart(){ unloadServo.setPosition(.744);}
    public void ringPullPivotServoOut(){ ringPullPivotServo.setPosition(0.0699);} //out --> add to initforAuto and the driving positions to the rest of the autos
    public void ringPullPivotServoIn(){ ringPullPivotServo.setPosition(0.4429);} //in
    public void ringPullArmServoUp(){ ringPullArmServo.setPosition(0.4119);} //up
    public void ringPullArmServoDown(){ ringPullArmServo.setPosition(0.5939);} //down
    public void initForAutonomous()
    {
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        //initialize hardware
        //main motors
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorFwdRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //other motors
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        intakeFwd = hardwareMap.get(DcMotor.class, "intakeFwd");
        intakeBack = hardwareMap.get(DcMotor.class, "intakeBack");
        //sensors
        distanceFwdLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdLeft");
        distanceFwdRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdRight");
        distanceBackLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBackLeft");
        distanceBackRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBackRight");
        distanceLeftTop = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeftTop");
        distanceLeftBottom = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeftBottom");
        //servos
        basketServo = hardwareMap.get(Servo.class,"basketServo");
        unloadServo = hardwareMap.get(Servo.class,"unloadServo");
        wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");
        wobbleGrabberServo = hardwareMap.get(Servo.class, "wobbleGrabberServo");
        ringPullPivotServo = hardwareMap.get(Servo.class, "ringPullPivotServo");
        ringPullArmServo = hardwareMap.get(Servo.class, "ringPullArmServo");
        resetDriveEncoders();
        // Lights turn green when it is calibrated
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        wobbleArmStart();
        wobbleGrabberClosed();
        unloadServoStart();
        basketServoDown();
        ringPullArmServoUp();
        ringPullPivotServoIn();

        //setup Camera
        final OpenCvCamera webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AutoSupplies.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        //--Setup Camera--
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1184,656, OpenCvCameraRotation.UPRIGHT);//320x240, 1024x576, 1184x656 all work -- 1280x720 does not for some reason
            }
        });
        double time1 = runtime.milliseconds();
        double time2 = 0;
        while(time2 < (time1 + 4000) && opModeIsActive()){//intended to ensure that the camera works but does not function
            time2 = runtime.milliseconds();
        }
        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(592,430);

        static final int REGION_WIDTH = 105;
        static final int REGION_HEIGHT = 150;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile AutoSupplies.SkystoneDeterminationPipeline.RingPosition position = AutoSupplies.SkystoneDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = AutoSupplies.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = AutoSupplies.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = AutoSupplies.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = AutoSupplies.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}
