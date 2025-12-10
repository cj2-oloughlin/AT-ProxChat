using UnityEngine;



namespace Kart
{
    public class AIInput : MonoBehaviour, IDrive
    {
        public Vector2 Move => Vector2.up;
        public bool IsBraking { get; }

        public void Enable()
        {
            //noop
        }
    }
}

        
        

    