// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

 public class PhotonCameraWrapper {
     public PhotonCamera photonCamera;
     public PhotonPoseEstimator photonPoseEstimator;
 
     public PhotonCameraWrapper(
        String cameraName, 
        Transform3d robotToCamera,
        PoseStrategy poseStrategy,
        AprilTagFieldLayout aprilTagFieldLayout
    ) {  
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            poseStrategy, 
            new PhotonCamera(cameraName), 
            robotToCamera
        );
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
      *     of the observation. Assumes a planar field and the robot is always firmly on the ground
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
         return photonPoseEstimator.update();
     }
 }