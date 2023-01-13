package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
    
    private boolean isThereTarget;
    private double distance;
    int targetID;
    double poseAmbiguity;
    PhotonCamera camera = new PhotonCamera("limelight");

    public void update(){
        var result = camera.getLatestResult();
        isThereTarget = result.hasTargets();

        if(isThereTarget){
            PhotonTrackedTarget target = result.getBestTarget();
            targetID = target.getFiducialId();
            poseAmbiguity = target.getPoseAmbiguity();
        }
    }

    public double getDistance() {
        return distance;
    }

    public boolean getIsThereTarget() {
      return isThereTarget;
    }
}
