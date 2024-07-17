package us.ihmc.psyonicControl;

import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.abilityhand.AbilityHandBLEManager;
import us.ihmc.abilityhand.AbilityHandIndividualFingerControlCommand;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.time.LocalDateTime;
import java.time.temporal.ChronoUnit;

public class psyonicController
{
   private final String[] handAddresses = new String[] {"DE:76:4F:34:6F:E1", "F9:C8:0F:A7:A4:D5"};
   private final AbilityHandIndividualFingerControlCommand leftHandControlCommand = new AbilityHandIndividualFingerControlCommand();
   private final AbilityHandIndividualFingerControlCommand rightHandControlCommand = new AbilityHandIndividualFingerControlCommand();
   AbilityHandBLEManager bleManager = new AbilityHandBLEManager(handAddresses);

   private final double[] leftPreviousAngles = new double[6];
   private final double[] rightPreviousAngles = new double[6];

   //All want to be yovariables
//   private final double tau = 0.05f;
//   private final double deadbandThreshold = 2.0f;
//   private final double MAXANGLE = 90.0f;
//   private final double MINANGLE = 0.0f;

   public static final double YO_VARIABLE_SERVER_UPDATE_PERIOD = 1 / 100.0;

   private YoVariableServer yoVariableServer;
   private Thread yoVariableUpdateThread;

   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble tau = new YoDouble("tau", yoRegistry);
   private final YoDouble deadbandThreshold = new YoDouble("deadbandThreshold", yoRegistry);
   private final YoDouble minAngle = new YoDouble("minAngle", yoRegistry);
   private final YoDouble maxAngle = new YoDouble("maxAngle", yoRegistry);

   public psyonicController()
   {
      tau.set(0.150);
      deadbandThreshold.set(12.17);
      minAngle.set(9.0);
      maxAngle.set(96);

      // start YoVariableServer
      yoVariableServer = new YoVariableServer("yoPsyonicServer", null, new DataServerSettings(false), YO_VARIABLE_SERVER_UPDATE_PERIOD);
      yoVariableServer.setMainRegistry(yoRegistry, null);
      LogTools.info("Starting YoVariableServer...");
      yoVariableServer.start();

      LogTools.info("Starting server update thread...");
      MutableLong timestamp = new MutableLong();

      yoVariableUpdateThread = new Thread(() ->
      {
         while (true)
         {
            yoVariableServer.update(timestamp.getAndAdd(Conversions.secondsToNanoseconds(YO_VARIABLE_SERVER_UPDATE_PERIOD)));

            try
            {
               Thread.sleep((long) Conversions.secondsToMilliseconds(YO_VARIABLE_SERVER_UPDATE_PERIOD));
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }
      }, "YoServerUpdate");

      yoVariableUpdateThread.start();
   }

   public int bluetoothConnect()
   {
      try
      {
         return bleManager.connect();
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }
   public void bluetoothDisconnect()
   {
      try
      {
         bleManager.disconnect();
      }
      catch(InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }
   public void setFingerSpeeds()
   {
      for (int i = 0; i < 6; i++) {
         leftPreviousAngles[i] = 0.0f;
         rightPreviousAngles[i] = 0.0f;
      }
      leftHandControlCommand.setThumbFlexorPeriod(0.0F);
      leftHandControlCommand.setThumbRotatorPeriod(0.0F);
      leftHandControlCommand.setIndexPeriod(0.0F);
      leftHandControlCommand.setMiddlePeriod(0.0F);
      leftHandControlCommand.setRingPeriod(0.0F);
      leftHandControlCommand.setPinkyPeriod(0.0F);
      rightHandControlCommand.setThumbFlexorPeriod(0.0F);
      rightHandControlCommand.setThumbRotatorPeriod(0.0F);
      rightHandControlCommand.setIndexPeriod(0.0F);
      rightHandControlCommand.setMiddlePeriod(0.0F);
      rightHandControlCommand.setRingPeriod(0.0F);
      rightHandControlCommand.setPinkyPeriod(0.0F);
   }
   public void setFingerAngles(double angle, int fingerNum, int x, int hand)
   {
      double filteredAngle;
      if (hand == 0) {
         filteredAngle = leftPreviousAngles[fingerNum] + tau.getValue() * (angle - leftPreviousAngles[fingerNum]);
      } else if (hand == 1){
         filteredAngle = rightPreviousAngles[fingerNum] + tau.getValue() * (angle - rightPreviousAngles[fingerNum]);
      }
      else {
         filteredAngle = 0;
      }
      if (Math.abs(filteredAngle - angle) < deadbandThreshold.getValue())
      {
         angle = filteredAngle;
      }
      if(fingerNum > 1)
      {
         if (angle > maxAngle.getValue())
         {
            angle = maxAngle.getValue();
         }
         else if (angle <= minAngle.getValue())
         {
            angle = minAngle.getValue();
         }
      }
      if(hand == 0)
      {
         leftPreviousAngles[fingerNum] = angle;
      }
      else if(hand == 1)
      {
         rightPreviousAngles[fingerNum] = angle;
      }
      if(fingerNum == 0)
      {
         if (x == 0)
         {
            if (angle <= minAngle.getValue())
            {
               angle = minAngle.getValue();
            }

            if(hand == 0)
            {
               leftHandControlCommand.setThumbFlexorPosition((float) angle);
            }
            else if(hand == 1)
            {
               angle *= 1.5;
               rightHandControlCommand.setThumbFlexorPosition((float) angle);
            }

         }
         else
         {
            if(hand == 0)
            {
               angle = -(100 - angle);
               if (angle > 0)
               {
                  angle = 0;
               }
               else if(angle < -74)
               {
                  angle = -80;
               }
               leftHandControlCommand.setThumbRotatorPosition((float) angle);
            }
            else if(hand == 1)
            {
               angle = angle - 100;
               if (angle > 0)
               {
                  angle = 0;
               }
               else if(angle < -120)
               {
                  angle = -120;
               }
               rightHandControlCommand.setThumbRotatorPosition((float) angle);
            }
         }
      }
      if(fingerNum == 1)
      {
         if(angle > maxAngle.getValue() - 5)
         {
            angle = maxAngle.getValue()-5;
         }
         if(hand == 0)
         {
            leftHandControlCommand.setIndexPosition((float) angle);
         }
         else if(hand == 1)
         {
            rightHandControlCommand.setIndexPosition((float) angle);
         }
      }
      else if(fingerNum == 2)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setMiddlePosition((float) angle);
         }
         else if(hand == 1)
         {
            rightHandControlCommand.setMiddlePosition((float) angle);
         }
      }
      else if(fingerNum == 3)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setRingPosition((float) angle);
         }
         else if(hand == 1)
         {
            rightHandControlCommand.setRingPosition((float) angle);
         }
      }
      else if(fingerNum == 4)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setPinkyPosition((float) angle);
         }
         else if(hand == 1)
         {
            rightHandControlCommand.setPinkyPosition((float) angle);
         }
      }
   }
   public void sendCommand()
   {
      bleManager.sendIndividualCommand(handAddresses[0], leftHandControlCommand);
      bleManager.sendIndividualCommand(handAddresses[1], rightHandControlCommand);
   }
}