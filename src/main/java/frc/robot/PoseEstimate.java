package frc.robot;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LimelightConstants;
public class PoseEstimate {

    public PhotonCamera limeLight;
    public RobotPoseEstimator robotPoseEstimator;

    public PoseEstimate(){
        
        AprilTagFieldLayout atfl;
        try {
            atfl = AprilTagFieldLayout.loadFromResource("2023-chargedup.json");
            limeLight = new PhotonCamera(LimelightConstants.cameraName);

            var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
            camList.add(new Pair<PhotonCamera, Transform3d>(limeLight, LimelightConstants.robotToCam));
    
            robotPoseEstimator = new RobotPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(
                    result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
}
