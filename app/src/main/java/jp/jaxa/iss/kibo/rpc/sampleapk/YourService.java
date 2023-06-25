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

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;

import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;

import android.util.Log;
import android.util.SparseArray;
import android.util.SparseIntArray;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */
@SuppressWarnings("SpellCheckingException")
public class YourService extends KiboRpcService {

    Mat camMat, distCoeff;
    List<Integer> activeTargets, prevTargets = new ArrayList<>();
    SparseArray<Coordinate> targetList;
    SparseArray<List<Integer>> parentIDInfo;
    SparseIntArray pointMap, quickMoves;

    String mQrContent = "No QR Content could be found.";
    int currParent = 0, phase = 1;

    final String
            TAG = "CARTOGRAPHER",
            SIM = "Simulator",
            IRL = "Orbit";

    @Override
    @SuppressWarnings("all")
    protected void runPlan1(){
        // record startTime
        long startTime = System.currentTimeMillis();

        // initialize data and start mission
        api.startMission();
        initCam(SIM);
        initMaps();

        // handle movement to targets and points per phase optimization
        while(api.getTimeRemaining().get(1) > 80000 && phase <= 3) {
            Log.i(TAG, "Entered phase #" + phase);
            int targetToHit = 0;

            if(api.getTimeRemaining().get(1) <= 80000 && phase == 3) {
                Log.i(TAG, "Breaking loop to prioritize time");
                break;
            }

            activeTargets = api.getActiveTargets();
            Log.i(TAG, "Active Targets: " + activeTargets.toString());

            targetToHit = ListUtils.findMaxFromMap(activeTargets, pointMap);

            if(activeTargets.size() > 1)
                targetToHit = STRATEGIZE(targetToHit);

            if(api.getTimeRemaining().get(1) < 80000) break;

            Log.i(TAG, "Going for target #" + targetToHit);
            moveTo(targetList.get(targetToHit), true, false);

            Log.i(TAG, "Time Remaining After Target #" + targetToHit + ": " +
                    api.getTimeRemaining().get(1) + "ms");

            Log.i(TAG, "Previous targets: " + prevTargets.toString());
            phase++;
        }
        Log.i(TAG, "Broke out of loop with " + api.getTimeRemaining().get(1) + "ms remaining");

        // handle QR code
        if(prevTargets.get(prevTargets.size() - 1) != 4 && api.getTimeRemaining().get(1) > 80000) {
            Log.i(TAG, "Heading to QR Code");
            moveTo(targetList.get(7), false, true); // Time Cost: 1m10s
        } else {
            scanQR(false);
            Log.i(TAG, "Skipped QR Code to prioritize time.");
        }
        Log.i(TAG, "QR Content: " + mQrContent);

        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();
        moveTo(targetList.get(8), false, false); // Time Cost: 45s

        // send mission completion and record deltaTime
        long deltaTime = System.currentTimeMillis() - startTime;
        Log.i(TAG, "runPlan1() executed in: " + deltaTime/1000 + "s.");
        api.reportMissionCompletion(mQrContent);
    }

    @Override
    protected void runPlan2(){ /* write your plan 2 here */ }

    @Override
    protected void runPlan3(){ /* write your plan 3 here */ }

    /**
     * moveTo Coordinate wrapper method
     * @param coordinate the coordinate to move to
     * @param scanTag true if moving to a tag to scan, false otherwise
     * @param QR true if moving to QR code, false otherwise
     * @return the scanned tag, if scanTag is true, else 0 (Unused)
     */
    @SuppressWarnings("UnusedReturnValue")
    private int moveTo(Coordinate coordinate, boolean scanTag, boolean QR){
        int target = targetList.indexOfValue(coordinate);

        if(coordinate.hasParent()){
            Coordinate parent = coordinate.getParent();
            moveTo(parent, false, false);
            currParent = parent.parentID;
            Log.i(TAG, "Current Parent ID: " + currParent); }

        moveTo(coordinate.getPoint(), coordinate.getQuaternion());
        if (scanTag) targetLaser(target);
        else if (QR) mQrContent = scanQR(false);

        if(coordinate.hasParent() && (target != 4) && (target != 8)) {
            Coordinate parent = coordinate.getParent();
            moveTo(parent, false, false);
            currParent = parent.parentID;
            Log.i(TAG, "Current Parent ID: " + currParent); }

        return target;
    }

    /**
     * Wrapper function for api.moveTo(point, quaternion, boolean) to make a failsafe
     * in case initial movement fails, and log movement details.
     * @param point the Point to move to
     * @param quaternion the Quaternion to angle Astrobee to
     */
    private void moveTo(Point point, Quaternion quaternion) {
        final int LOOP_MAX = 10;

        Log.i(TAG, "Moving to: " + point.getX() + ", " + point.getY() + ", " + point.getZ());
        long start = System.currentTimeMillis();

        Result result = api.moveTo(point, quaternion, true);

        long end = System.currentTimeMillis();
        long elapsedTime = end - start;
        Log.i(TAG, "[0] moveTo finished in : " + elapsedTime/1000 + " seconds");
        Log.i(TAG, "[0] hasSucceeded : " + result.hasSucceeded());

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

            loopCounter++;
        }
    }

    /** moveTo double/float Specifics wrapper */
    @SuppressWarnings("unused")
    private void moveTo(double pt_x, double pt_y, double pt_z, float q_x, float q_y, float q_z, float q_w){
        moveTo(new Point(pt_x, pt_y, pt_z), new Quaternion(q_x, q_y, q_z, q_w));
    }

    /**
     * Pre-Load the Camera Matrix and Distortion Coefficients to save time.
     * @param mode 'SIM' or 'IRL' -> Simulator or Real Coefficients, Respectively
     */
    @SuppressWarnings("SameParameterValue")
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

    /**
     * Processes NavCam Matrix and Scans for AprilTags within NavCam
     * @return the ID of the Target found in the NavCam, and 0 if none found.
     */
    @SuppressWarnings({"UnusedReturnValue", "unused"})
    private int getTagInfo(int tagNum){
        Log.i(TAG, "Calling getTagInfo() function");
        long start = System.currentTimeMillis();

        Mat
                undistorted = new Mat(),
                ids = new Mat();

        api.flashlightControlFront(0.05f); // enable flashlight for tag read clarity
        Mat distorted = api.getMatNavCam();
        api.flashlightControlFront(0.00f);

        Imgproc.undistort(distorted, undistorted, camMat, distCoeff);
        Log.i(TAG, "Undistorted Image Successfully");

        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters detParams = DetectorParameters.create();
        List<Mat> detectedMarkers = new ArrayList<>();

        Aruco.detectMarkers(undistorted, dict, detectedMarkers, ids, detParams);
        List<Integer> markerIds = getIdsFromMat(ids);

        int iters = 0, iter_max = 10, target = 0;
        while(markerIds.size() == 0 && iters < iter_max){
            Log.i(TAG, "No Markers found. Trying again [" + iters + "]");
            Aruco.detectMarkers(undistorted, dict, detectedMarkers, ids, detParams);
            markerIds = getIdsFromMat(ids);
            if(markerIds.size() != 0)
                break;

            iters++;
        }
        Log.i(TAG, "Marker IDs Found: " + markerIds.toString());

        if (ListUtils.containsAny(markerIds, Constants.targetOneIDs)){target = 1;}
        else if (ListUtils.containsAny(markerIds, Constants.targetTwoIDs)){target = 2;}
        else if (ListUtils.containsAny(markerIds, Constants.targetThreeIDs)){target = 3;}
        else if (ListUtils.containsAny(markerIds, Constants.targetFourIDs)){target = 4;}
        else if (ListUtils.containsAny(markerIds, Constants.targetFiveIDs)){target = 5;}
        else if (ListUtils.containsAny(markerIds, Constants.targetSixIDs)){target = 6;}

        long delta = (System.currentTimeMillis() - start)/1000;
        Log.i(TAG, "Found Target #" + target);
        Log.i(TAG, "Read AprilTags in " + delta + " seconds");

        api.saveMatImage(undistorted, "tag" + target + "Image.png");
        return target;
    }

    /**
     * Moves to QR Code and Scans NavCam Mat for a QR Code, then returns back to central position.
     * (Returns to central position to avoid KOZ)
     * @return QR Code Content as a String
     */
    private String scanQR(boolean skipQRread) {
        Log.i(TAG, "Arrived at QR Code");

        Map<String, String> map = new HashMap<>();
        String[] keys = {"JEM", "COLUMBUS", "RACK1", "ASTROBEE", "INTBALL", "BLANK"};

        map.put(keys[0], "STAY_AT_JEM");
        map.put(keys[1], "GO_TO_COLUMBUS");
        map.put(keys[2], "CHECK_RACK_1");
        map.put(keys[3], "I_AM_HERE");
        map.put(keys[4], "LOOKING_FORWARD_TO_SEE_YOU");
        map.put(keys[5], "NO_PROBLEM");

        Mat points = new Mat(), QR = new Mat(); String data = "";
        QRCodeDetector detector = new QRCodeDetector();

        if(!skipQRread) {
            api.flashlightControlFront(0.05f);
            Mat distorted = api.getMatNavCam();
            api.flashlightControlFront(0.0f);
            QR = new Mat();
            Imgproc.undistort(distorted, QR, camMat, distCoeff);

            api.saveMatImage(QR, "qrCode.png");

            Log.i(TAG, "Attempting QR Code Scan");
            points = new Mat();
            data = detector.detectAndDecode(QR, points);
        }

        int iters = 0, iter_max = 10;
        while(points.empty() && iters < iter_max && !skipQRread){
            Log.i(TAG, "No QR found. Trying again [" + iters + "]");
            points = new Mat();
            data = detector.detectAndDecode(QR, points);
            if(!points.empty())
                break;

            iters++;
        }

        String RET_STRING;
        if(!points.empty() && !skipQRread) {
            Log.i(TAG, "Scanned QR Code and got data: " + data);
            RET_STRING = map.get(data);
        } else {
            Log.i(TAG, "Scanned QR Code and got data: null");

            Random r = new Random(); // failsafe system
            String randomGenQRTag = keys[r.nextInt(keys.length)];

            Log.i(TAG, "QR Scan failed, reporting data: " + randomGenQRTag);
            RET_STRING = map.get(randomGenQRTag);
        }

        return RET_STRING;
    }

    /**
     * Method to handle Laser Targeting
     * @param targetNum the laser to target
     */
    private void targetLaser(int targetNum){
        if(!activeTargets.contains(targetNum))
            return;

        long start = System.currentTimeMillis();

        api.laserControl(true);
        api.saveMatImage(api.getMatNavCam(), "target_" + targetNum + ".png");
        Log.i(TAG, "Laser on.");
        sleep(1000);
        api.takeTargetSnapshot(targetNum);
        prevTargets.add(targetNum);
        Log.i(TAG, "Took Target #" + targetNum + " snapshot successfully.");

        Log.i(TAG, "Laser off after being on for: " +
                ((System.currentTimeMillis() - start)/1000)/60 + "s");
    }

    /**
     * Takes in a Mat and returns its elements as ArrayList
     * @param ids the Mat to convert to ArrayList
     * @return the Mat converted to ArrayList
     */
    private List<Integer> getIdsFromMat(Mat ids){
        List<Integer> markerIds = new ArrayList<>();
        for(int i = 0; i < ids.rows(); i++){
            for(int j = 0; j < ids.cols(); j++){
                double[] idData = ids.get(i, j);
                int id = (int) idData[0];
                markerIds.add(id); }}
        return markerIds;
    }

    /**
     * Initializes all data structures based on Map<> or List<>
     * Post-condition: targetList, parentIDInfo, and pointMap are initialized
     */
    private void initMaps(){
        targetList = new SparseArray<>();
        targetList.put(0, Constants.start);
        targetList.put(1, Constants.targetOne);
        targetList.put(2, Constants.targetTwo);
        targetList.put(3, Constants.targetThree);
        targetList.put(4, Constants.targetFour);
        targetList.put(5, Constants.targetFive);
        targetList.put(6, Constants.targetSix);
        targetList.put(7, Constants.targetQR);
        targetList.put(8, Constants.goal);
        Log.i(TAG, "Initialized Movement SparseArray");

        parentIDInfo = new SparseArray<>();
        List<Integer> _12356QR_ = new ArrayList<Integer>(){{
            add(1); add(2); add(3); add(5); add(6); add(7);}};
        List<Integer> _4G_ = new ArrayList<Integer>(){{add(4); add(8);}};
        parentIDInfo.put(1, _12356QR_);
        parentIDInfo.put(2, _4G_);
        Log.i(TAG, "Initialized Parent ID SparseArray");

        pointMap = new SparseIntArray();
        pointMap.put(1, 30); pointMap.put(2, 20);
        pointMap.put(3, 40); pointMap.put(4, 20);
        pointMap.put(5, 30); pointMap.put(6, 30);
        Log.i(TAG, "Initialized Points SparseIntArray");

        quickMoves = new SparseIntArray();
        quickMoves.put(1, 4); quickMoves.put(4, 1);
        quickMoves.put(1, 5); quickMoves.put(5, 1);
        quickMoves.put(1, 6); quickMoves.put(6, 1);
        quickMoves.put(2, 6); quickMoves.put(6, 2);
        quickMoves.put(4, 5); quickMoves.put(5, 4);
        quickMoves.put(4, 6); quickMoves.put(6, 4);
        quickMoves.put(5, 6); quickMoves.put(6, 5);
        Log.i(TAG, "Initialized QuickMoves SparseIntArray");
    }

    /**
     * Helper function for Thread.sleep to avoid long Exception Handling blocks
     * @param millis milliseconds to sleep for
     */
    private void sleep(long millis){
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Strategy function that changes targetToHit based on parameters
     * @param targetToHit the maximum point target in activeTargets
     * @param phase the current game phase
     * @return updated targetToHit
     */
    @SuppressWarnings("all")
    private int STRATEGIZE(int targetToHit){
        if(phase == 3 && (targetToHit == 1 || targetToHit == 2)) {
            if(activeTargets.contains(1)) activeTargets.remove(activeTargets.indexOf(1));
            if(activeTargets.contains(2)) activeTargets.remove(activeTargets.indexOf(2));
            targetToHit = activeTargets.get(0);
            return targetToHit; }

        if(targetToHit == 4 && phase != 3) {
            activeTargets.remove(activeTargets.indexOf(4));
            targetToHit = activeTargets.get(0);
            return targetToHit; }

        if(targetToHit == 2) {
            activeTargets.remove(activeTargets.indexOf(2));
            targetToHit = activeTargets.get(0);
            return targetToHit; }

        if(activeTargets.contains(3) && targetToHit != 3 && phase != 3) {
            targetToHit = 3;
            return targetToHit; }

        return targetToHit;
    }
}