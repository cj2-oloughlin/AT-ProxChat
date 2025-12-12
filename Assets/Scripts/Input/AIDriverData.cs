using UnityEngine;



namespace Kart
{
    [CreateAssetMenu(fileName = "AIDriverData", menuName = "Kart/AIDriverData")]

    public class AIDriverData : ScriptableObject
    {
        public float proximityThreshold = 20.0f; //Required distance for a waypoint to be classed as visited
        public float updateCornerRange = 50.0f; //Distance from a corner before the waypoint is updated
        public float brakeRange = 80.0f; //Distance from a corner before the kart will brake
        public float spinThreshold = 100.0f; //Angular velocity where the AI will start counter-steering
        public float speedWhileDrifting = 0.5f; //Driving speed while drifting
        public float timeToDrift = 0.5f; //Time holding drift
    }
}

        
        

    