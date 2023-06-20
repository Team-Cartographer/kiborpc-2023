package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import java.util.List;

class Constants {

    public static final Coordinate start = new Coordinate(
            new Point(9.815, -9.806, 4.293), new Quaternion(1, 0, 0, 0)
    );

    public static final Coordinate goal = new Coordinate(
            new Point(11.143, -6.7607, 4.9654), new Quaternion(0, 0, -0.707f, 0.707f)
    );

    public static final Coordinate pointOne = new Coordinate(
            new Point(11.2746, -9.92284, 5.2988), new Quaternion(0f, 0f, -0.707f, 0.707f)
    );

    public static final Coordinate pointTwo = new Coordinate(
            new Point(10.612, -9.0709, 4.48), new Quaternion(0.5f, 0.5f, -0.5f, -0.5f)
    );

    public static final Coordinate pointThree = new Coordinate(
            new Point(10.71, -7.7, 4.48), new Quaternion(0, 0.707f, 0, 0.707f)
    );

    public static final Coordinate pointFour = new Coordinate(
            new Point(10.51, -6.7185, 5.1804), new Quaternion(0, 0, -1, 0)
    );

    public static final Coordinate pointFive = new Coordinate(
            new Point(11.114, -7.9756, 5.3393), new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f)
    );

    public static final Coordinate pointSix = new Coordinate(
            new Point(11.355, -8.9929, 4.7818), new Quaternion(0, 0, 0, 1)
    );

    public static final Coordinate pointSeven = new Coordinate(
            new Point(11.369, -8.5518, 4.48), new Quaternion(0f, 0.707f, 0f, 0.707f)
    );

    // TODO figure out what the target Coordinates really mean on the map!
    public static final Coordinate targetOne = new Coordinate(
            new Point(11.2625, -10.58, 5.3625), new Quaternion(0.707f, 0f, 0f, 0.707f)
    );

    public static final Coordinate targetTwo = new Coordinate(
            new Point(10.513384, -9.085172, 3.76203), new Quaternion(0f, 0f, 0f, 1f)
    );

    public static final Coordinate targetThree = new Coordinate(
            new Point(10.6031, -7.71007, 3.76093), new Quaternion(0.707f, 0f, 0f, 0.707f)
    );

    public static final Coordinate targetFour = new Coordinate(
            new Point(9.866984, -6.673972, 5.09531), new Quaternion(-0.5f, 0.5f, -0.5f, 0.5f)
    );

    public static final Coordinate targetFive = new Coordinate(
            new Point(11.102, -8.0304, 5.9076), new Quaternion(1f, 0f, 0f, 0f)
    );

    public static final Coordinate targetSix = new Coordinate(
            new Point(12.023, -8.989, 4.8305), new Quaternion(0.5f, 0.5f, -0.5f, -0.5f)
    );

    public static final Coordinate QRCode = new Coordinate(
            new Point(11.381944, -8.566172, 3.76203), new Quaternion(0f, 0f, 0f, 1f)
    );

    public static final LaserTarget TARGET_ONE = new LaserTarget(5);
    public static final LaserTarget TARGET_TWO = new LaserTarget(6);
    public static final LaserTarget TARGET_THREE = new LaserTarget(4);
    public static final LaserTarget TARGET_FOUR  = new LaserTarget(6);
    public static final LaserTarget TARGET_FIVE = new LaserTarget(5);
    public static final LaserTarget TARGET_SIX = new LaserTarget(5);

    public static final int[] targetOneIDs = {1, 2, 3, 4};
    public static final int[] targetTwoIDs = {5, 6, 7, 8};
    public static final int[] targetThreeIDs = {9, 10, 11, 12};
    public static final int[] targetFourIDs = {13, 14, 15, 16};
    public static final int[] targetFiveIDs = {17, 18, 19, 20};
    public static final int[] targetSixIDs = {21, 22, 23, 24};
}

class Coordinate {
    private Point point;
    private Quaternion quaternion;

    Coordinate(Point pt, Quaternion qt) {
        point = pt;
        quaternion = qt;
    }

    Point getPoint() {return point;}
    Quaternion getQuaternion() {return quaternion;}

    public void setPoint(Point pt) {point = pt;}
    public void setQuaternion(Quaternion qt) {quaternion = qt;}
}

class LaserTarget {
    public float
            height,
            width,
            dotRadius,
            distEdgeToApriltag,
            distVertCenterApriltagToCenterDot,
            distHorizCenterApriltagToCenterDot;

    // all units are in centimeters
    LaserTarget(float dotRadius) {
        height = 15f;
        width = 27f;
        distEdgeToApriltag = 1.25f;
        distVertCenterApriltagToCenterDot = 3.75f;
        distHorizCenterApriltagToCenterDot = 10f;
        this.dotRadius = dotRadius;
    }
}

class ListUtils {
    public static boolean containsAny(List<Integer> arrayList, int[] elements) {
        for (int element : elements) {
            if (arrayList.contains(element)) {
                return true;
            }
        }
        return false;
    }
}