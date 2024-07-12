package us.ihmc.Examples;

import us.ihmc.remoteCaptury.remoteCapturyControl;

public class remoteCapturyExample
{
   private static final remoteCapturyControl remoteCaptury = new remoteCapturyControl();
   private static boolean running = true;
   public static void main(String[] args) throws InterruptedException
   {
      remoteCaptury.loadLibrary();
      remoteCaptury.startConnection();
      Runtime.getRuntime().addShutdownHook(new Thread(() ->
                                                      {
                                                         running = false;
                                                         remoteCaptury.stopConnection();
                                                      }));
      while(running)
      {
         Thread.sleep(50);
         remoteCaptury.updatePose();
      }
   }
}
