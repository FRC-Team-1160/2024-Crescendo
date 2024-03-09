package frc.robot.subsystems.Vision;

import java.util.ArrayList;

import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class Vision extends SubsystemBase{
    
  private static Vision m_instance;
  public PhotonCamera m_shooterCamera;
  public PhotonCamera m_backCamera;
  public Pose3d m_pose;
  public DriveTrain m_drive;

  public Pose3d tracked_note;
  public ArrayList<Point> noteCenters;
  Timer tracking_timeout;

  PhotonPoseEstimator m_photonPoseEstimator;
  StructPublisher<Pose3d> adv_posePub;
  StructArrayPublisher<Pose3d> adv_targetPub;
  StructPublisher<Pose3d> adv_trackedPub;

  public static Vision getInstance(){
    if (m_instance == null){
      m_instance = new Vision();
    }
    return m_instance;
  }

  public Vision(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_vision = inst.getTable("adv_vision");
    adv_posePub = adv_vision.getStructTopic("Pose", Pose3d.struct).publish();
    adv_targetPub = adv_vision.getStructArrayTopic("Target", Pose3d.struct).publish();
    adv_trackedPub = adv_vision.getStructTopic("Tracked", Pose3d.struct).publish();

    m_shooterCamera = new PhotonCamera("Camera_Module_v1");//Camera_Module_v1
    m_backCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    m_drive = DriveTrain.getInstance();
        
    m_pose = new Pose3d(12.5, 0, 0, new Rotation3d());

    m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_shooterCamera, new Transform3d());
    m_photonPoseEstimator.setReferencePose(m_pose);

    tracked_note = null;
    tracking_timeout = new Timer();
    tracking_timeout.start();
  }

  @Override
  public void periodic(){
    var result1 = m_shooterCamera.getLatestResult();
    if (result1.hasTargets()){
      var update = m_photonPoseEstimator.update();
      if (update.isPresent()){
        m_pose = update.get().estimatedPose;
        if (Math.abs(m_pose.getZ()) < 1){
          m_photonPoseEstimator.setReferencePose(m_pose);
        }
        if (m_drive != null){
          m_drive.m_poseEstimator.addVisionMeasurement(m_pose.toPose2d(), Timer.getFPGATimestamp());
        }
      }
    }
    adv_posePub.set(m_pose);

    double min_dist = 99;

    if (m_drive.odomPose != null && noteCenters != null){
      Pose2d odomPose = m_drive.odomPose;
      double rot = odomPose.getRotation().getRadians();
      ArrayList<Pose3d> poses3d = new ArrayList<Pose3d>();
      for (Point p : noteCenters){
        double f = 0.76 / (Math.tan(38 / 2 * Math.PI/180) * (p.y - 60)/60);
        double h = -f * Math.tan(63 / 2 * Math.PI/180) * (p.x - 80)/80;
        double x = f * Math.cos(rot) - h * Math.sin(rot);
        double y = f * Math.sin(rot) + h * Math.cos(rot);
        double dist = Math.sqrt(f*f + h*h);
        Pose3d pose = new Pose3d(
          x + odomPose.getX(),
          y + odomPose.getY(),
          0.04,
          new Rotation3d()
        );
        poses3d.add(pose);
        if (dist < min_dist){
          min_dist = dist;
          tracked_note = pose;
          tracking_timeout.reset();
        }
      }
      adv_targetPub.set(poses3d.toArray(new Pose3d[0]));
      adv_trackedPub.set(tracked_note);
    } else {
      if (tracking_timeout.hasElapsed(0.5)){
        tracked_note = null;
      }
    }
    

  }
  
}
