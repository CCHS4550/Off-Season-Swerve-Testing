package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * creates a simulation version of vision, using the same logic as the normal photonvision hardware,
 * except now we use an imaginary camera
 */
public class VisionIOSim extends VisionIOPhotonvision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(Constants.VisionConstants.aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();

    cameraProperties.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProperties.setAvgLatencyMs(35);
    cameraProperties.setLatencyStdDevMs(5);
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
