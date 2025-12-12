using UnityEngine;



namespace Kart
{
    public class AIInput : MonoBehaviour, IDrive
    {
        public Circuit circuit;
        public AIDriverData driverData;

        public Vector2 Move => Vector2.up;
        public bool IsBraking { get; }

        public void Enable()
        {
            //noop
        }


        //Keeps track of the AI's progress
        int currentWaypointIndex;
        int currentCornerIndex;

        float previousYaw; //For comparing the difference in the Kart's angular velocity

        public void AddDriverData(AIDriverData data) => driverData = data;
        public void AddCircuit(Circuit circuit) => this.circuit = circuit;
    }
}

        
        

    