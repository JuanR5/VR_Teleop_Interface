 using UnityEngine;  
   
 public class ControllerVibrationManager : MonoBehaviour  
 {  
     // Public methods to trigger vibration  
     public void TriggerVibration(OVRInput.Controller controller, float amplitude, float duration)  
     {  
        Debug.Log($"TriggerVibration called: controller={controller}, amplitude={amplitude}, duration={duration}"); // Add this line
        OVRInput.SetControllerVibration(duration, amplitude, controller);  
        Invoke("StopVibration", duration);  
     }  
   
     private void StopVibration()  
     {  
         OVRInput.SetControllerVibration(0, 0, OVRInput.Controller.RTouch);  
         OVRInput.SetControllerVibration(0, 0, OVRInput.Controller.LTouch);  
     }  
 }  