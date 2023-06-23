package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

class Constants {
    
    /*PARENT MOVEMENTS***/
     private static final Coordinate parentOneTwo = new Coordinate(
            new Point(10.612, -9.079, 5.25), new Quaternion(0, 0, 0, 0)
     );
    
     private static final Coordinate parentFourGoal = new Coordinate(
            new Point(10.51, -6.7185, 5.25), new Quaternion(0, 0, 0, 0)
     );

     private static final Coordinate parentThreeFiveSixQR = new Coordinate(
            new Point(11.369, -8.5518, 5.25), new Quaternion(0, 0, 0, 0)
     );
    
     /*TARGET MOVEMENTS***/
     static final Coordinate start = new Coordinate(
            new Point(10.4, -10, 4.4), new Quaternion(0, 0, 0, 0)
     );

     static final Coordinate targetOne = new Coordinate(
            new Point(11.2746, -9.92284, 5.2988), new Quaternion(0, 0, -0.707f, 0.707f),
             parentOneTwo
     );

     static final Coordinate targetTwo = new Coordinate(
            new Point(10.612, -9.0709, 4.48), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f),
             parentOneTwo
     );

     static final Coordinate targetThree = new Coordinate(
            new Point(10.71, -7.7, 4.48), new Quaternion(0, 0.707f, 0, 0.707f),
             parentThreeFiveSixQR
     );

     static final Coordinate targetFour = new Coordinate(
            new Point(10.51, -6.7185, 5.1804), new Quaternion(0, 0.707f, 0, 0.707f),
             parentFourGoal
     );

     static final Coordinate targetFive = new Coordinate(
            new Point(11.114, -7.9756, 5.3393), new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f),
             parentThreeFiveSixQR
     );

     static final Coordinate targetSix = new Coordinate(
            new Point(11.355, -8.9929, 4.7818), new Quaternion(0, 0, 0, 1),
             parentThreeFiveSixQR
     );

     static final Coordinate targetQR = new Coordinate(
             new Point(11.369, -8.5518, 4.40), new Quaternion(0.707f, 0f, -0.707f, 0f),
             parentThreeFiveSixQR
     );
     static final Coordinate goal = new Coordinate(
             new Point(11.143, -6.7607, 4.48), new Quaternion(0, 0, -0.707f, 0.707f),
             parentFourGoal
     );
    
    /*OTHER CONSTANTS**/
//    **Currently Unused, but needed for Laser Targeting**
//    static final LaserTarget TARGET_ONE = new LaserTarget(5);
//    static final LaserTarget TARGET_TWO = new LaserTarget(6);
//    static final LaserTarget TARGET_THREE = new LaserTarget(4);
//    static final LaserTarget TARGET_FOUR  = new LaserTarget(6);
//    static final LaserTarget TARGET_FIVE = new LaserTarget(5);
//    static final LaserTarget TARGET_SIX = new LaserTarget(5);
    
    /*TARGET IDS**/
    static final int[] targetOneIDs = {1, 2, 3, 4};
    static final int[] targetTwoIDs = {5, 6, 7, 8};
    static final int[] targetThreeIDs = {9, 10, 11, 12};
    static final int[] targetFourIDs = {13, 14, 15, 16};
    static final int[] targetFiveIDs = {17, 18, 19, 20};
    static final int[] targetSixIDs = {21, 22, 23, 24};
}

class Coordinate {
    private Point point;
    private Quaternion quaternion;
    private Coordinate parent;
    private boolean has_parent;

    Coordinate(Point pt, Quaternion qt) {
        point = pt;
        quaternion = qt;
        has_parent = false;

    }
    
    Coordinate(Point pt, Quaternion qt, Coordinate prt){
        point = pt;
        quaternion = qt; 
        parent = prt;
        has_parent = true;
    }

    Point getPoint() {return point;}
    Quaternion getQuaternion() {return quaternion;}
    Coordinate getParent() {return parent;}
    
    boolean hasParent() {return has_parent;}

    @SuppressWarnings("unused")
    void setPoint(Point pt) {point = pt;}
    @SuppressWarnings("unused")
    void setQuaternion(Quaternion qt) {quaternion = qt;}
    @SuppressWarnings("unused")
    void setParent(Coordinate prt) {parent = prt; has_parent = true;}
}

//@SuppressWarnings("unused")
//class LaserTarget {
//     float height,
//            width,
//            dotRadius,
//            distEdgeToApriltag,
//            distVertCenterApriltagToCenterDot,
//            distHorizCenterApriltagToCenterDot;
//
//    // all units are in centimeters
//    LaserTarget(float dotRadius) {
//        height = 15f;
//        width = 27f;
//        distEdgeToApriltag = 1.25f;
//        distVertCenterApriltagToCenterDot = 3.75f;
//        distHorizCenterApriltagToCenterDot = 10f;
//        this.dotRadius = dotRadius;
//    }
//}
