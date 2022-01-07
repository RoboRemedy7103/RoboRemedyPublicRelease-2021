package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import edu.wpi.first.math.util.Units;

/*
 * Photon vision system control.
 */
public class PhotonVision {

    public enum PhotonPipeline {
        TestOne, TestTwo
    }

    static PhotonCamera photon = new PhotonCamera("CameraOne");
    static PhotonPipeline currentPhotonPipeline = PhotonPipeline.TestOne;
    static int currentPipeline = 0;

    public static void setPipeline(PhotonPipeline pipeline) {
        if (pipeline == currentPhotonPipeline) {
            return;
        }

        if (pipeline == PhotonPipeline.TestOne) {
            currentPipeline = 1;
        } else if (pipeline == PhotonPipeline.TestTwo) {
            currentPipeline = 2;
        }

        currentPhotonPipeline = pipeline;
        photon.setPipelineIndex(currentPipeline);
    }

    public static void setLEDMode(VisionLEDMode mode) {
        photon.setLED(mode);
    }

    public static boolean isTargetFound() {
        var result = photon.getLatestResult();

        boolean isTargetFound = false;
        isTargetFound = result.hasTargets();
        return isTargetFound;
    }

    public static double calcDist(double cameraHeightInches, double targetHeightInches, double cameraPitchRadians) {
        double cameraHeightMeters = cameraHeightInches * 0.0254;
        double targetHeightMeters = targetHeightInches * 0.0254;
        double dist = PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters, cameraPitchRadians, getTargetPitchRadians());
        dist = dist * 39.3701;
        return dist;
    }

    public static double getTargetPitchRadians(){
        var result = photon.getLatestResult();
        if (result.hasTargets()) {
            var targetResult = result.getBestTarget();
            if (targetResult != null) {
                double pitch = Units.degreesToRadians(targetResult.getPitch());
                return pitch;
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    }
}