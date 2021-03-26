package org.firstinspires.ftc.teamcode.Autonomous;
//-----imports-----
//main imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//opencv imports
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
//webcam imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="Red Right Auto", group="CompetitionAuto")
//@Disabled
public class RedRightAuto extends AutoSupplies{
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();

        //  Wait until start
        waitForStart();
        //-----Add Code Here: Refer to Blue Left Auto once it is done with coding-----
    }
}
