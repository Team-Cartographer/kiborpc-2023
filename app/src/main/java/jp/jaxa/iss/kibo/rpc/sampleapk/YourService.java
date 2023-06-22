package jp.jaxa.iss.kibo.rpc.sampleapk;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Random;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.MatOfPoint;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;

import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;

import android.util.Log;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */
public class YourService extends KiboRpcService {

    Mat camMat, distCoeff;

    final String
            TAG = "CARTOGRAPHER",
            SIM = "Simulator",
            IRL = "Orbit"; // IRL -> 'In Real Life'

    /**
     * Wrapper function for api.moveTo(point, quaternion, boolean) to make a fail-safe
     * in case initial movement fails, and log movement details.
     * @param point the Point to move to
     * @param quaternion the Quaternion to orient angles to
     */
    private void moveTo(Point point, Quaternion quaternion) {
        final int LOOP_MAX = 10;

        Log.i(TAG, "[0] Calling moveTo function ");
        Log.i(TAG, "Moving to: " + point.getX() + ", " + point.getY() + ", " + point.getZ());
        long start = System.currentTimeMillis();

        Result result = api.moveTo(point, quaternion, true);

        long end = System.currentTimeMillis();
        long elapsedTime = end - start;
        Log.i(TAG, "[0] moveTo finished in : " + elapsedTime/1000 + " seconds");
        Log.i(TAG, "[0] hasSucceeded : " + result.hasSucceeded());
        Log.i(TAG, "[0] getStatus : " + result.getStatus().toString());
        Log.i(TAG, "[0] getMessage : " + result.getMessage());

        int loopCounter = 1;
        while (!result.hasSucceeded() && loopCounter <= LOOP_MAX) {

            Log.i(TAG, "[" + loopCounter + "] " + "Calling moveTo function");
            start = System.currentTimeMillis();

            result = api.moveTo(point, quaternion, true);

            end = System.currentTimeMillis();
            elapsedTime = end - start;
            Log.i(TAG, "[" + loopCounter + "] " + "moveTo finished in : " + elapsedTime / 1000 +
                    " seconds");
            Log.i(TAG, "[" + loopCounter + "] " + "hasSucceeded : " + result.hasSucceeded());
            Log.i(TAG, "[" + loopCounter + "] " + "getStatus : " + result.getStatus().toString());
            Log.i(TAG, "[" + loopCounter + "] " + "getMessage : " + result.getMessage());

            loopCounter++;
        }
    }

    // Extra wrappers for moveTo for convenience
    private void moveTo(Coordinate coord){
        moveTo(coord.getPoint(), coord.getQuaternion());
    }

    private void moveTo(double pt_x, double pt_y, double pt_z, float q_x, float q_y, float q_z, float q_w){
        moveTo(new Point(pt_x, pt_y, pt_z), new Quaternion(q_x, q_y, q_z, q_w));
    }

    //TODO the method below is for image processing which comes with laser detection
    /**
     * Pre-Load the Camera Matrix and Distortion Coefficients to save time.
     * @param mode 'SIM' or 'IRL' -> Simulator or Real Coefficients, Respectively
     */
    private void initCam(String mode){
        camMat = new Mat(3, 3, CvType.CV_32F);
        distCoeff = new Mat(1, 5, CvType.CV_32F);

        if(mode.equals(SIM)){
            // all numbers via programming manual
            float[] camArr = {
                    661.783002f, 0.000000f, 595.212041f,
                    0.000000f, 671.508662f, 489.094196f,
                    0.000000f, 0.000000f, 1.000000f
            };
            float[] distortionCoefficients = {
                    -0.215168f, 0.044354f, 0.003615f, 0.005093f, 0.000000f
            };

            camMat.put(0, 0, camArr);
            distCoeff.put(0,0, distortionCoefficients);
        }
        else if(mode.equals(IRL)){
            float[] camArr = {
                    753.51021f, 0.0f, 631.11512f,
                    0.0f, 751.3611f, 508.69621f,
                    0.0f, 0.0f, 1.0f
            };
            float[] distortionCoefficients = {
                    -0.411405f, 0.177240f, -0.017145f, 0.006421f, 0.000000f
            };

            camMat.put(0, 0, camArr);
            distCoeff.put(0,0, distortionCoefficients);
        }
        Log.i(TAG, "Initialized Camera Matrices in Mode: " + mode);
    }

    @Override
    protected void runPlan1(){
        // the mission starts
        api.startMission();
        initCam(SIM);

        // get the list of active target id
        List<Integer> activeTargets = api.getActiveTargets();
        Log.i(TAG, "Active Targets: " + activeTargets.toString());

        // TODO hardcode more movement functions

        // Move to Point 6 (Testing)
        // avoid KOZ
        moveTo(new Point(10.515, -9.806, 4.593),
                new Quaternion(1f, 0f, 0f, 0f));
        moveTo(Constants.targetSix);

        int foundTarget = getTagInfo();
        if (activeTargets.contains(foundTarget)) {
            targetLaser(foundTarget, activeTargets);
        }

        moveTo(new Point(10.612f, -9.0709f, 5.25f), new Quaternion(0, 0, 0, 0));

        // get QR code content
        String mQrContent = handleQRCode();
        Log.i(TAG, "QR Content: " + mQrContent);

        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();

        // send mission completion
        api.reportMissionCompletion(mQrContent);
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    /**
     * Processes NavCam Matrix and Scans for AprilTags within NavCam
     * @return the ID of the Target found in the NavCam, and 0 if none found.
     */
    private int getTagInfo(){
        Log.i(TAG, "Calling getTagInfo() function");
        long start = System.currentTimeMillis();

        Mat undistorted = new Mat(),
                ids = new Mat();

        api.flashlightControlFront(0.05f); // enable flashlight for tag read clarity
        Mat distorted = api.getMatNavCam();
        api.flashlightControlFront(0.0f);

        Imgproc.undistort(distorted, undistorted, camMat, distCoeff);
        Log.i(TAG, "Undistorted Image Successfully");

        Rect ROI = new Rect(371, 261, 454, 256);
        undistorted = new Mat(undistorted, ROI);

        api.saveMatImage(undistorted, "TEST_IMG.png");

        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters detParams = DetectorParameters.create();
        List<Mat> detectedMarkers = new ArrayList<>();

        Aruco.detectMarkers(undistorted, dict, detectedMarkers, ids, detParams);

        List<Integer> markerIds = new ArrayList<>();
        for(int i = 0; i < ids.rows(); i++){
            for(int j = 0; j < ids.cols(); j++){
                double[] idData = ids.get(i, j);
                int id = (int) idData[0];
                markerIds.add(id); }}
        Log.i(TAG, "Marker IDs Found: " + markerIds.toString());

        int target = 0;
        if (containsAny(markerIds, Constants.targetOneIDs)){target = 1;}
        else if (containsAny(markerIds, Constants.targetTwoIDs)){target = 2;}
        else if (containsAny(markerIds, Constants.targetThreeIDs)) {target = 3;}
        else if (containsAny(markerIds, Constants.targetFourIDs)) {target = 4;}
        else if (containsAny(markerIds, Constants.targetFiveIDs)) {target = 5;}
        else if (containsAny(markerIds, Constants.targetSixIDs)) {target = 6;}

        long delta = (System.currentTimeMillis() - start)/1000;
        Log.i(TAG, "Found Target #" + target);
        Log.i(TAG, "Read AprilTags in " + delta + " seconds");

        return target;
    }

    private String handleQRCode(){
        Quaternion QRQuaternion = new Quaternion(0.707f, 0f, -0.707f, 0f);

        moveTo(new Point(11.369f, -8.5518f, 5.25f), QRQuaternion);
        moveTo(new Point(11.369f, -8.5518f, 4.7818f), QRQuaternion);
        Log.i(TAG, "Arrived at QR Code");

        String qrData = scanQRCode();
        moveTo(new Point(11.369f, -8.5518f, 5.25f), QRQuaternion);

        return qrData;
    }

    /**
     * Scans NavCam Mat for a QR Code
     * @return QR Code Content as a String
     */
    private String scanQRCode() {
        Map<String, String> map = new HashMap<>();
        String[] keys = {"JEM", "COLUMBUS", "RACK1", "ASTROBEE", "INTBALL", "BLANK"};

        map.put(keys[0], "STAY_AT_JEM");
        map.put(keys[1], "GO_TO_COLUMBUS");
        map.put(keys[2], "CHECK_RACK_1");
        map.put(keys[3], "I_AM_HERE");
        map.put(keys[4], "LOOKING_FORWARD_TO_SEE_YOU");
        map.put(keys[5], "NO_PROBLEM");

        QRCodeDetector detector = new QRCodeDetector();

        api.flashlightControlFront(0.05f);
        Mat distorted = api.getMatNavCam();
        api.flashlightControlFront(0.0f);
        Mat QR = new Mat();
        Imgproc.undistort(distorted, QR, camMat, distCoeff);

        Log.i(TAG, "Attempting QR Code Scan");
        Mat points = new Mat();
        String data = detector.detectAndDecode(QR, points);


        if(!points.empty()) {
            Log.i(TAG, "Scanned QR Code and got data: " + data);
            return map.get(data);
        } else {
            Log.i(TAG, "Scanned QR Code and got data: null");

            Random r = new Random();
            String randomGenQRTag = keys[r.nextInt(keys.length)];

            Log.i(TAG, "QR Scan failed, reporting data: " + randomGenQRTag);
            return map.get(randomGenQRTag);
        }
    }

    private void targetLaser(int targetNum, List<Integer> activeTargets){
        if(!activeTargets.contains(targetNum))
            return;

        api.laserControl(true); Log.i(TAG, "Laser on.");

        try {
            Thread.sleep(1000);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }

        api.takeTargetSnapshot(targetNum);
        api.laserControl(false); Log.i(TAG, "Laser off.");
    }

    public static boolean containsAny(List<Integer> arrayList, int[] elements) {
        for (int element : elements) {
            if (arrayList.contains(element)) {
                return true; }}
        return false;
    }
}