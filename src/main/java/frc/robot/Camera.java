package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
    
    private boolean isThereTarget;
    private double distance;
    int targetID;
    PhotonCamera camera = new PhotonCamera("limelight");

    public void update(){
        var result = camera.getLatestResult();
        isThereTarget = result.hasTargets();

        if(isThereTarget){
            PhotonTrackedTarget target = result.getBestTarget();
            targetID = target.getFiducialId();
        }
    }

    public boolean getIsThereTarget() {
      return isThereTarget;
    }
}
