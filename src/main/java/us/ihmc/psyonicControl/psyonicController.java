package us.ihmc.psyonicControl;

import us.ihmc.abilityhand.AbilityHandBLEManager;
import us.ihmc.abilityhand.AbilityHandIndividualFingerControlCommand;

public class psyonicController
{
   private final String[] handAddresses = new String[] {"DE:76:4F:34:6F:E1", "F9:C8:0F:A7:A4:D5"};
   private final AbilityHandIndividualFingerControlCommand leftHandControlCommand = new AbilityHandIndividualFingerControlCommand();
   private final AbilityHandIndividualFingerControlCommand rightHandControlCommand = new AbilityHandIndividualFingerControlCommand();
   AbilityHandBLEManager bleManager = new AbilityHandBLEManager(handAddresses);

   private final float[] leftPreviousAngles = new float[6];
   private final float[] rightPreviousAngles = new float[6];

   private final float tau = 0.1f;

   private final float deadbandThreshold = 8.0f;
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
   public void setFingerAngles(float angle, int fingerNum, int x, int hand)
   {float filteredAngle;
      if (hand == 0) {
         filteredAngle = leftPreviousAngles[fingerNum] + tau * (angle - leftPreviousAngles[fingerNum]);
      } else if (hand == 1){
         filteredAngle = rightPreviousAngles[fingerNum] + tau * (angle - rightPreviousAngles[fingerNum]);
      }
      else {
         filteredAngle = 0;
      }

      if (Math.abs(filteredAngle - angle) < deadbandThreshold) {
         angle = filteredAngle;
      }

      if(angle > 150)
      {
         angle = 150;
      }
      else if (angle <= 0)
      {
         angle = 0;
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
            if(hand == 0)
            {
               leftHandControlCommand.setThumbFlexorPosition(angle);
            }
            if(hand == 1)
            {
               rightHandControlCommand.setThumbFlexorPosition(angle);
            }

         }
         else
         {
            if(hand == 0)
            {
               angle = -(80 - angle);
               if (angle > 0)
               {
                  angle = 0;
               }
               else if (angle < -80)
               {
                  angle = -80;
               }

               leftHandControlCommand.setThumbRotatorPosition(angle);
            }
            if(hand == 1)
            {
               angle = angle - 100;
               if (angle > 0)
               {
                  angle = 0;
               }
               if (angle < -100)
               {
                  angle = -100;
               }
               rightHandControlCommand.setThumbRotatorPosition(angle);
            }
         }
      }
      if(fingerNum == 1)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setIndexPosition(angle);
         }
         if(hand == 1)
         {
            rightHandControlCommand.setIndexPosition(angle);
         }
      }
      else if(fingerNum == 2)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setMiddlePosition(angle);
         }
         if(hand == 1)
         {
            rightHandControlCommand.setMiddlePosition(angle);
         }
      }
      else if(fingerNum == 3)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setRingPosition(angle);
         }
         if(hand == 1)
         {
            rightHandControlCommand.setRingPosition(angle);
         }
      }
      else if(fingerNum == 4)
      {
         if(hand == 0)
         {
            leftHandControlCommand.setPinkyPosition(angle);
         }
         if(hand == 1)
         {
            rightHandControlCommand.setPinkyPosition(angle);
         }
      }
   }
   public void sendCommand()
   {
      bleManager.sendIndividualCommand(handAddresses[0], leftHandControlCommand);
      bleManager.sendIndividualCommand(handAddresses[1], rightHandControlCommand);
   }
}