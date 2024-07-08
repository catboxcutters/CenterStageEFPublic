package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

//import org.openftc.easyopencv.

public class CaseDetectionPipeline
{
    public  OpenCvWebcam webcam;
    Point topLeft,topRight,topMid,bottomLeft,bottomMid,bottomRight
            ,topMidLeft,topMidRight,
            bottomMidLeft,bottomMidRight;
    Point midLeft;
    Point midRight;
    public static int rezX = 1920;
    public static int rezY = 1080;
    int mean1,mean2,mean3;
    int v1=0,v2=0,v3=0;
    public int Case=0;
    public String mode="";
    int action;
    public int tresh = 170;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public CaseDetectionPipeline(HardwareMap map, Telemetry tel,String _m){
        hardwareMap = map;
        telemetry = tel;
        mode = _m;
    }

    public int getCase(){

        if(v1 < v2){
            if(v2 < v3){
                return  3;
            }
            else{
                return  2;
            }
        } else {
            if(v1<v3){
                return  3;
            }
            else {
                return  1;
            }
        }


    }

    public void initCamera(){
        topLeft = new Point(0 ,0);
        // topMid= new Point(rezX/3 + 1,0);
        topRight= new Point(rezY/3 * 2 + 1,0);
        topMidLeft = new Point(0,rezX/3);
        // midMid = new Point(rezX/3 +1,rezY/3);
        topMidRight = new Point(rezY,rezX/3 );

        bottomMidLeft = new Point(0,rezX/3 * 2);
        bottomMidRight = new Point(rezY,rezX/3 *2);

        bottomLeft = new Point(0,rezX);
        //bottomMid= new Point(rezX/3 * 2,rezY - rezY/2);
        bottomRight= new Point(rezY,rezX);
        //midLeft = new Point(0,0);
        //midRight = new Point(rezX-1,rezX-1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        // webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDevice();


/*
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(rezX, rezY, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("ok");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData()
            }
        });

 */

    }

    void sleep(int ms){
        try{
            Thread.sleep(ms);
        }catch (InterruptedException e){

        }
    }

    public void recognize(){
        sleep(1000);
        webcam.stopStreaming();
        webcam.stopRecordingPipeline();
    }

    public  void stop(){
        webcam.stopStreaming();
        webcam.stopRecordingPipeline();
    }

    public void start(int c){
        action = c;
        if(action == 1) {
            // webcam.setPipeline(new SamplePipeline());
        } else if(action == 2) {
            //webcam.setPipeline(new DetectorPipeLine());
        }
        webcam.startStreaming(rezX, rezY, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }


    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        public Mat Channel = new Mat();
        public Mat Channel2 = new Mat();
        public Mat reg1, reg2, reg3;

        @Override
        public Mat processFrame(Mat input)
        {
            imageProc(input);
            Mat binary = new Mat(Channel2.rows(), Channel2.cols(), Channel2.type(), new Scalar(0));
            //Imgproc.threshold(Channel2, binary, tresh, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(Channel2, binary, tresh, 255, Imgproc.THRESH_BINARY);

            reg1 = binary.submat(new Rect(topLeft, topMidRight));
            reg2 = binary.submat(new Rect(topMidLeft, bottomMidRight));
            reg3 = binary.submat(new Rect(bottomMidLeft, bottomRight));

            //reg1 = binary.submat(new Rect(midLeft,midRight));

            mean1 = (int) (Core.mean(reg1).val[0]);
            mean2 = (int) (Core.mean(reg2).val[0]);
            mean3 = (int) (Core.mean(reg3).val[0]);

            v1 = mean1;
            if(mode == "red") {
                Imgproc.rectangle(input, topLeft, topMidRight, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, topMidLeft, bottomMidRight, new Scalar(255, 0, 0), 3);
                Imgproc.rectangle(input, bottomMidLeft, bottomRight, new Scalar(255, 0, 0), 3);
            }
            if(mode == "blue") {
                Imgproc.rectangle(input, topLeft, topMidRight, new Scalar(0, 0, 255), 3);
                Imgproc.rectangle(input, topMidLeft, bottomMidRight, new Scalar(0, 0, 255), 3);
                Imgproc.rectangle(input, bottomMidLeft, bottomRight, new Scalar(0, 0, 255), 3);
            }
//                Imgproc.rectangle(input, topLeft, topMidRight, new Scalar(12, 106, 179), 3);
//                Imgproc.rectangle(input, topMidLeft, bottomMidRight, new Scalar(0, 255, 0), 3);
//                Imgproc.rectangle(input, bottomMidLeft, bottomRight, new Scalar(255, 0, 0), 3);



            v2 = mean2;
            v3 = mean3;

            /*
            3 stanga
            2 fata
            1 dreapta
             */


            Case = getCase();

            return binary;

        }

        void imageProc(Mat frame) {
            Imgproc.cvtColor(frame,Channel,Imgproc.COLOR_RGB2YCrCb);
            if(mode == "blue") {
                Core.extractChannel(Channel, Channel2, 2);
            }
            else if(mode=="red"){
                Core.extractChannel(Channel, Channel2,1);
            }
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}

/*
class DetectorPipeLine extends OpenCvPipeline
{
    boolean viewportPaused;
    public Mat Channel = new Mat();
    public Mat Channel2 = new Mat();
    public Mat reg1, reg2, reg3;

    @Override
    public Mat processFrame(Mat input)
    {
        imageProc(input);
        Mat binary = new Mat(Channel2.rows(), Channel2.cols(), Channel2.type(), new Scalar(0));
        Imgproc.threshold(Channel2, binary, 150, 255, Imgproc.THRESH_BINARY_INV);
        reg1 = binary.submat(new Rect(topLeft, bottomLeft));
        reg2 = binary.submat(new Rect(topMid, bottomMid));
        reg3 = binary.submat(new Rect(topRight, bottomRight));

        mean1 = (int) (Core.mean(reg1).val[0]);
        mean2 = (int) (Core.mean(reg2).val[0]);
        mean3 = (int) (Core.mean(reg3).val[0]);

        v1 = mean1;
        v2 = mean2;
        v3 = mean3;


        Case = getCase();

        return binary;
    }

    void imageProc(Mat frame) {
        Imgproc.cvtColor(frame,Channel,Imgproc.COLOR_RGB2YCrCb);
        if(mode == "blue") {
            Core.extractChannel(Channel, Channel2, 2);
        }
        else if(mode=="red"){
            Core.extractChannel(Channel, Channel2,0);
        }
    }

    @Override
    public void onViewportTapped()
    {
        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }
}
 */

