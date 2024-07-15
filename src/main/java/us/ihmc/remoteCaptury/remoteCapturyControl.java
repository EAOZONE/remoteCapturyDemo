package us.ihmc.remoteCaptury;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.psyonicControl.psyonicController;
import us.ihmc.remotecaptury.CapturyActor;
import us.ihmc.remotecaptury.CapturyPose;
import us.ihmc.abilityhand.*;
import us.ihmc.remotecaptury.library.RemoteCapturyNativeLibrary;

import static us.ihmc.remotecaptury.global.remotecaptury.*;

public class remoteCapturyControl
{
   // According to people at capturyLive transforms are the same list and order as joints
   private final float GLOBALSIZECHANGE = 0.001F;
   private final String[] jointNames = {"Root", "Hips", "Spine", "Spine1", "Spine2", "Spine3", "Spine4", "Neck", "Head", "HeadEE", "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand", "LeftHandThumb1", "LeftHandThumb2", "LeftHandThumb3", "LeftHandThumbEE", "LeftHandIndex1", "LeftHandIndex2", "LeftHandIndex3", "LeftHandIndexEE", "LeftHandMiddle1", "LeftHandMiddle2", "LeftHandMiddle3", "LeftHandMiddleEE", "LeftHandRing1", "LeftHandRing2", "LeftHandRing3", "LeftHandRingEE", "LeftHandPinky1", "LeftHandPinky2", "LeftHandPinky3", "LeftHandPinkyEE", "LeftHandEE", "RightShoulder", "RightArm", "RightForeArm", "RightHand", "RightHandThumb1", "RightHandThumb2", "RightHandThumb3", "RightHandThumbEE", "RightHandIndex1", "RightHandIndex2", "RightHandIndex3", "RightHandIndexEE", "RightHandMiddle1", "RightHandMiddle2", "RightHandMiddle3", "RightHandMiddleEE", "RightHandRing1", "RightHandRing2", "RightHandRing3", "RightHandRingEE", "RightHandPinky1", "RightHandPinky2", "RightHandPinky3", "RightHandPinkyEE", "RightHandEE", "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "LeftFootEE", "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase", "RightFootEE"
   };
   private final int[] parentNum = {-1, -1, 1, 2, 3, 4, 5, 6, 7, 8, 5, 10, 11, 12, 13, 14, 15, 16, 13, 18, 19, 20, 13, 22, 23, 24, 13, 26, 27, 28, 13, 30, 31, 32, 13, 5, 35, 36, 37, 38, 39, 40, 41, 38, 43, 44, 45, 38, 47, 48, 49, 38, 51, 52, 53, 38, 55, 56, 57, 38, 1, 60, 61, 62, 63, 1, 65, 66, 67, 68};
   private final RigidBodyTransform[] transforms = new RigidBodyTransform[jointNames.length];
   private final ReferenceFrame[] referenceFrames = new ReferenceFrame[jointNames.length];
   private final FramePose3D[] framePoses = new FramePose3D[jointNames.length];
   private final int[] fingerTransformNum = {14, 18, 22, 26, 30};
   private final int[] rightFingerTransformNum = {39, 43, 47, 51, 54};
   private final int transformNum = 0;
   private int ACTOR_ID = 30000;
   private static CapturyPose pose;
   private final psyonicController psyonicControl = new psyonicController();
   private int numOfHands = 0;

   public remoteCapturyControl() {
      for (int i = 0; i < jointNames.length; i++) {
         transforms[i] = new RigidBodyTransform();
         framePoses[i] = new FramePose3D();
      }
   }

   private static void connect()
   {
      // Connects to CapturyLive on the other computer as well as the 8 cameras
      System.out.println("Connecting...");
      Captury_connect("172.16.66.239", (short) 2101);
      Captury_startStreamingImages(CAPTURY_STREAM_IMAGES, 0xa36391a4);
      Captury_startStreamingImages(CAPTURY_STREAM_IMAGES, 0xa363947a);
      Captury_startStreamingImages(CAPTURY_STREAM_IMAGES, 0xa3639485);
      Captury_startStreamingImages(CAPTURY_STREAM_IMAGES, 0xa36429bc);
      Captury_startStreamingImages(CAPTURY_STREAM_IMAGES, 0xa36429bd);
      Captury_startStreamingImages(CAPTURY_STREAM_IMAGES, 0xa36429be);
      Captury_startStreamingImages(CAPTURY_STREAM_IMAGES, 0xa36429c0);
      Captury_startStreamingImages(CAPTURY_STREAM_IMAGES, 0xa36429c2);
      Captury_startStreaming(CAPTURY_STREAM_SCALES);
      Captury_startStreaming(CAPTURY_STREAM_LOCAL_POSES);
   }

   public void loadLibrary(){
      RemoteCapturyNativeLibrary.load();
   }

   public void startConnection(int newACTOR_ID)
   {
      ACTOR_ID = newACTOR_ID;
      // Makes sure the computer is disconnect before running everything else
      Captury_stopStreaming();
      Captury_disconnect();
      while(Captury_getConnectionStatus()!= CAPTURY_DISCONNECTED){
         Captury_stopStreaming();
         Captury_disconnect();
      }
      // Start tracking
      connect();
      while(numOfHands < 2)
      {
         numOfHands = psyonicControl.bluetoothConnect();
         System.out.println(numOfHands);
      }
      System.out.println("Number of hands connected: " + numOfHands);
      psyonicControl.setFingerSpeeds();
      Captury_startTracking(ACTOR_ID, 0, 0, 720);
   }

   public void getActor()
   {
      // Keeps trying to connect to the actor until an actor is found
      // Usually in the 4th "Snapping Actor" print statement
      // Initialize actor
      CapturyActor actors = new CapturyActor();
      while (Captury_getActorStatus(ACTOR_ID) == ACTOR_UNKNOWN)
      {
         try
         {
            if (Captury_getConnectionStatus() == CAPTURY_CONNECTED)
            {
               System.out.println("Snapping Actor");
               Captury_snapActor(0, 0, 720);
               //In miliseconds C++ code was in seconds
               Thread.sleep(3000);
               Captury_getActors(actors);
            }
            else
            {
               connect();
               Thread.sleep(1000);
            }
         }
         catch(InterruptedException e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   public void stopConnection()
   {
      // Disconnects from All connected software
      System.out.println("Disconnecting");
      psyonicControl.bluetoothDisconnect();
      stopTracking(); // TODO: does this need to come before deleteActor?
      Captury_stopStreaming();
      Captury_disconnect();
   }
   public void stopTracking()
   {
      Captury_deleteActor(ACTOR_ID);
      Captury_stopTracking(ACTOR_ID);
   }
   public boolean getPersonStatus()
   {
      return Captury_getActorStatus(ACTOR_ID) == ACTOR_TRACKING;
   }
   private void updateTransforms()
   {
      // Gets the Pose from CapturyLive
      pose = Captury_getCurrentPose(ACTOR_ID);
      if (pose != null)
      {
         for (int i = 0; i < fingerTransformNum.length; i++)
         {
            if (fingerTransformNum[i] == 14)
            {
               psyonicControl.setFingerAngles(2 * pose.transforms().getPointer(fingerTransformNum[i]).rotation().get(0), i, 0, 0);
               psyonicControl.setFingerAngles(Math.abs(2 * pose.transforms().getPointer(fingerTransformNum[i]).rotation().get(1)), i, 1, 0);
            }
            else
            {
               float average = calculateAverage(fingerTransformNum[i], 2);
               psyonicControl.setFingerAngles(Math.abs(2 * average), i, 2, 0);
            }
         }
         for (int i = 0; i < rightFingerTransformNum.length; i++)
         {
            if (rightFingerTransformNum[i] == 39)
            {

               psyonicControl.setFingerAngles(2 * pose.transforms().getPointer(rightFingerTransformNum[i]).rotation().get(0), i, 0, 1);
               psyonicControl.setFingerAngles(Math.abs(2 * pose.transforms().getPointer(rightFingerTransformNum[i]).rotation().get(1)), i, 1, 1);
            }
            else
            {
               float average = calculateAverage(rightFingerTransformNum[i], 2);
               psyonicControl.setFingerAngles(Math.abs(2 * average), i, 2, 1);
            }
         }
         //      for(int j = 0; j < jointNames.length; j++)
         //      {
         //         // converts the euler angles to quaternion
         //         float[] quat = new float[4];
         //         float[] euler = {pose.transforms().getPointer(j + transformNum).rotation().get(0), pose.transforms().getPointer(j + transformNum).rotation().get(1), pose.transforms().getPointer(j + transformNum).rotation().get(2)};
         //         liveEulerToQuaternion(euler, quat);
         //
         //         RigidBodyTransform transform = transforms[j];
         //         transform.getRotation().setQuaternion(quat[3], quat[1], quat[2], quat[0]);
         //         transform.getTranslation()
         //                  .setX((pose.transforms().getPointer(j + transformNum).translation().get(2))
         //                        * GLOBALSIZECHANGE);
         //         transform.getTranslation()
         //                  .setY((pose.transforms().getPointer(j + transformNum).translation().get(0))
         //                        * GLOBALSIZECHANGE);
         //         transform.getTranslation()
         //                  .setZ((pose.transforms().getPointer(j + transformNum).translation().get(1))
         //                        * GLOBALSIZECHANGE);
         //      }
         //   private void updateFrames() {
         //      for (int j = 0; j < jointNames.length; j++) {
         //         if (j <=1) {
         //            referenceFrames[j] = ReferenceFrameTools.constructFrameWithChangingTransformToParent(jointNames[j], ReferenceFrame.getWorldFrame(), transforms[j]);
         //         } else {
         //            referenceFrames[j] = ReferenceFrameTools.constructFrameWithChangingTransformToParent(jointNames[j], referenceFrames[parentNum[j] - transformNum], transforms[j]);
         //         }
         //      }
         //   }
      }
   }

   public void updatePose(){
      updateTransforms();
//      updateFrames();
      psyonicControl.sendCommand();
//      for (int i = 0; i < jointNames.length; i++)
//      {
//         // Sets the framePose with respect to the reference frame
//         // Then converts to world frame
//         FramePose3D framePose = framePoses[i];
//         framePose.setToZero(referenceFrames[i]);
//         framePose.changeFrame(ReferenceFrame.getWorldFrame());
//      }
   }
//   private static void liveEulerToMatrix(float[] euler, float[] m4x4) {
//      float c3 = (float) Math.cos(Math.toRadians(euler[0]));
//      float s3 = (float) Math.sin(Math.toRadians(euler[0]));
//      float c2 = (float) Math.cos(Math.toRadians(euler[1]));
//      float s2 = (float) Math.sin(Math.toRadians(euler[1]));
//      float c1 = (float) Math.cos(Math.toRadians(euler[2]));
//      float s1 = (float) Math.sin(Math.toRadians(euler[2]));
//
//      m4x4[0]  = c1*c2;         m4x4[1]  = c1*s2*s3 - c3*s1;  m4x4[2]  = s1*s3 + c1*c3*s2;  m4x4[3]  = 0.0f;
//      m4x4[4]  = c2*s1;         m4x4[5]  = c1*c3 + s1*s2*s3;  m4x4[6]  = c3*s1*s2 - c1*s3;  m4x4[7]  = 0.0f;
//      m4x4[8]  = -s2;           m4x4[9]  = c2*s3;             m4x4[10] = c2*c3;             m4x4[11] = 0.0f;
//      m4x4[12] = 0.0f;          m4x4[13] = 0.0f;              m4x4[14] = 0.0f;              m4x4[15] = 1.0f;
//   }
//   private static void liveMatrixToQuaternion(float[] m4x4, float[] quat) {
//      float trace = m4x4[0] + m4x4[5] + m4x4[10];
//      if (trace > 0.0f) {
//         float s = 0.5f / (float)Math.sqrt(trace + 1.0f);
//         quat[0] = 0.25f / s;
//         quat[1] = (m4x4[9] - m4x4[6]) * s;
//         quat[2] = (m4x4[2] - m4x4[8]) * s;
//         quat[3] = (m4x4[4] - m4x4[1]) * s;
//      } else {
//         if (m4x4[0] > m4x4[5] && m4x4[0] > m4x4[10]) {
//            float s = 2.0f * (float)Math.sqrt(1.0f + m4x4[0] - m4x4[5] - m4x4[10]);
//            quat[0] = (m4x4[9] - m4x4[6]) / s;
//            quat[1] = 0.25f * s;
//            quat[2] = (m4x4[1] + m4x4[4]) / s;
//            quat[3] = (m4x4[2] + m4x4[8]) / s;
//         } else if (m4x4[5] > m4x4[10]) {
//            float s = 2.0f * (float)Math.sqrt(1.0f + m4x4[5] - m4x4[0] - m4x4[10]);
//            quat[0] = (m4x4[2] - m4x4[8]) / s;
//            quat[1] = (m4x4[1] + m4x4[4]) / s;
//            quat[2] = 0.25f * s;
//            quat[3] = (m4x4[6] + m4x4[9]) / s;
//         } else {
//            float s = 2.0f * (float)Math.sqrt(1.0f + m4x4[10] - m4x4[0] - m4x4[5]);
//            quat[0] = (m4x4[4] - m4x4[1]) / s;
//            quat[1] = (m4x4[2] + m4x4[8]) / s;
//            quat[2] = (m4x4[6] + m4x4[9]) / s;
//            quat[3] = 0.25f * s;
//         }
//      }
//   }
//   private static void liveEulerToQuaternion(float[] euler, float[] quat) {
//      float[] mat = new float[16];
//      liveEulerToMatrix(euler, mat);
//      liveMatrixToQuaternion(mat, quat);
//   }

   private float calculateAverage(int initialTransformNum, int num)
   {
      // Used for psyonic hands
      float sum = 0;
      for(int i = initialTransformNum; i < initialTransformNum + 4; i++)
      {
         sum += Math.abs(pose.transforms().getPointer(i).rotation().get(num));
      }
      sum/=4;
      return sum;
   }

   public void setACTOR_ID(int ACTOR_ID)
   {
      this.ACTOR_ID = ACTOR_ID;
   }
}