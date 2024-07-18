package us.ihmc.Examples;

import us.ihmc.remoteCaptury.remoteCapturyControl;

public class remoteCapturyExample
{
   private static final remoteCapturyControl remoteCaptury = new remoteCapturyControl();
   private static boolean running = true;
   private static int ACTOR_ID = 30000;
   public static void main(String[] args) throws InterruptedException
   {
      remoteCaptury.loadLibrary();
      Runtime.getRuntime().addShutdownHook(new Thread(()->{
         running = false;
         remoteCaptury.stopConnection();
   }));
      remoteCaptury.startConnection(ACTOR_ID);
      while(running)
      {
         remoteCapturyTrackingAndHandControl();
         ACTOR_ID++;
         remoteCaptury.setACTOR_ID(ACTOR_ID);
      }
   }
   private static void remoteCapturyTrackingAndHandControl() throws InterruptedException
   {
      remoteCaptury.getActor();
      Thread.sleep(1000);
      while(remoteCaptury.getPersonStatus())
      {
         Thread.sleep(50);
         remoteCaptury.updatePose();
      }
      remoteCaptury.stopTracking();
   }
}



