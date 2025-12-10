using System;
using UnityEngine;
using Utilities;
using System.Linq;
using UnityEngine.InputSystem;
using static PlayerInputActions;


namespace Kart
{
    [System.Serializable]
    public class AxleInfo
    {
        public WheelCollider leftWheel;
        public WheelCollider rightWheel;
        public bool Motor; //Is torque being applied
        public bool steering; //Can wheels turn
        public WheelFrictionCurve originalForwardFriction;
        public WheelFrictionCurve originalSidewaysFriction;
    }






    public class KartController : MonoBehaviour
    {
        [Header("Axle Information")]
        [SerializeField] AxleInfo[] axleInfos;

        [Header("Motor Attributes")]
        [SerializeField] float maxMotorTorque = 3000f;
        [SerializeField] float maxSpeed;

        [Header("Steering Attributes")]
        [SerializeField] float maxSteeringAngle = 30f;
        [SerializeField] AnimationCurve turncurve;
        [SerializeField] float turnStrength = 1500f;

        [Header("Braking And Drifting Attributes")]
        [SerializeField] float driftSteerMultiplier = 1.5f; //Steering while Drifting
        [SerializeField] float brakeTorque = 10000f;

        [Header("Physics")]
        [SerializeField] Transform centerOfMass;
        [SerializeField] float downForce = 100f;
        [SerializeField] float gravity = Physics.gravity.y;
        [SerializeField] float lateralGScale = 10f; //Scaling factor

        [Header("Banking Attributes")]
        [SerializeField] float maxBankAngle = 5f;
        [SerializeField] float bankSpeed = 2f;

        [Header("References")]
        [SerializeField] InputReader playerInput;
        [SerializeField] IDrive input;
        Rigidbody rb;


        Vector3 kartVelocity;
        float brakeVelocity;
        float driftVelocity;

        RaycastHit hit;

        const float thresholdSpeed = 10f;
        const float centerOfMassOffset = -0.5f;
        Vector3 originalCenterOfMass;

        public bool isGrounded;
        public Vector3 Velocity => kartVelocity;
        public float MaxSpeed => maxSpeed;


        private void Awake()
        {
            if (playerInput is IDrive driveInput)
            {
                input = driveInput;
            }
            else
            {
                Debug.Log("Using AI Kart Input System");
                var aiInput = gameObject.AddComponent<AIInput>();
                //Configures the AI kart input
                input = aiInput;

            }
        }

        void Start()
        {
            rb = GetComponent<Rigidbody>();
            input.Enable();

            rb.centerOfMass = centerOfMass.localPosition;
            originalCenterOfMass = centerOfMass.localPosition;


            foreach (AxleInfo axleInfo in axleInfos)
            {
                axleInfo.originalForwardFriction = axleInfo.leftWheel.forwardFriction;
                axleInfo.originalSidewaysFriction = axleInfo.leftWheel.sidewaysFriction;
            }
        }


        private void FixedUpdate()
        {
            float verticalInput = AdjustInput(input.Move.y);
            float horizontalInput = AdjustInput(input.Move.x);

            float motor = maxMotorTorque * verticalInput;
            float steering = maxSteeringAngle * horizontalInput;

            UpdateAxles(motor, steering);
            UpdateBanking(horizontalInput);

            kartVelocity = transform.InverseTransformDirection(rb.linearVelocity);

            if (isGrounded)
            {
                HandleGroundedMovement(verticalInput, horizontalInput);
            }
            else
            {
                HandleAirborneMovement(verticalInput, horizontalInput);
            }
        }

        void HandleGroundedMovement(float verticalInput, float horizontalInput)
        {
            //Turning - Picks a point on the defined curve based on the kart's speed
            if (Mathf.Abs(verticalInput) > 0.1f || Mathf.Abs(kartVelocity.z) > 1)
            {
                float turnMultiplier = Mathf.Clamp01(turncurve.Evaluate(kartVelocity.magnitude / maxSpeed));
                rb.AddTorque(Vector3.up * horizontalInput * Mathf.Sign(kartVelocity.z) * turnStrength * 100f * turnMultiplier);
            }

            //Acceleration - Increments the player's speed (while normalizing it) until the maximum speed is achieved
            if (!input.IsBraking)
            {
                float targetSpeed = verticalInput * maxSpeed;
                Vector3 forwardWithoutY = transform.forward.With(y: 0).normalized;
                rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, forwardWithoutY * targetSpeed, Time.deltaTime);
            }

            //Downforce - Keeps the kart down while accounting for speed - uses lateral Gs to scale if the kart drifts
            float speedFactor = Mathf.Clamp01(rb.linearVelocity.magnitude / maxSpeed);
            float lateralG = Mathf.Abs(Vector3.Dot(rb.linearVelocity, transform.right));
            float downForceFactor = Mathf.Max(speedFactor, lateralG / lateralGScale);
            rb.AddForce(-transform.up * downForce * rb.mass * downForceFactor);

            //Shift Center of Mass - Adjusting the Physics of the kart
            float speed = rb.linearVelocity.magnitude;
            Vector3 centerOfMassAdjustment = (speed > thresholdSpeed)
                ? new Vector3(0f, 0f, Mathf.Abs(verticalInput) > 0.1f ? Mathf.Sign(verticalInput) * centerOfMassOffset : 0f)
                : Vector3.zero;
            rb.centerOfMass = originalCenterOfMass + centerOfMassAdjustment;




        }

        void UpdateBanking(float horizontalInput) //Bank the kart when the player is in a turn
        {
            float targetBankAngle = horizontalInput * -maxBankAngle;
            Vector3 currentEuler = transform.localEulerAngles;
            currentEuler.z = Mathf.LerpAngle(currentEuler.z, targetBankAngle, Time.deltaTime * bankSpeed);
            transform.localEulerAngles = currentEuler;
        }

        void HandleAirborneMovement(float verticalInput, float horizontalInput) //Applies gravity when the kart isn't grounded
        {
            rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, rb.linearVelocity + Vector3.down * gravity, Time.deltaTime * gravity);
        }

        void UpdateAxles(float motor, float steering)
        {
            foreach (AxleInfo axleInfo in axleInfos)
            {
                HandleSteering(axleInfo, steering);
                HandleMotor(axleInfo, motor);
                HandleBreaksAndDrift(axleInfo); 
                UpdateWheelVisuals(axleInfo.leftWheel);
                UpdateWheelVisuals(axleInfo.rightWheel);

            }
        }

        void UpdateWheelVisuals(WheelCollider collider)
        {
            if (collider.transform.childCount == 0) return;

            Transform visualWheel = collider.transform.GetChild(0);

            Vector3 position;
            Quaternion rotation;
            collider.GetWorldPose(out position, out rotation);

            visualWheel.transform.position = position;
            visualWheel.transform.rotation = rotation;
        }




        void HandleSteering(AxleInfo axleInfo, float steering)
        {
            if(axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
        }

        void HandleMotor(AxleInfo axleInfo, float motor)
        {
            if (axleInfo.Motor)
            {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
        }

        void HandleBreaksAndDrift(AxleInfo axleInfo)
        {
            if (axleInfo.Motor)
            {
                if (input.IsBraking)
                {
                    rb.constraints = RigidbodyConstraints.FreezeRotationX;

                    float newZ = Mathf.SmoothDamp(rb.linearVelocity.z, 0, ref brakeVelocity, 1f);
                    rb.linearVelocity = rb.linearVelocity.With(newZ);

                    axleInfo.leftWheel.brakeTorque = brakeTorque;
                    axleInfo.rightWheel.brakeTorque = brakeTorque;
                    ApplyDriftFriction(axleInfo.leftWheel);
                    ApplyDriftFriction(axleInfo.rightWheel);
                }
                else
                {
                    rb.constraints = RigidbodyConstraints.None;

                    axleInfo.leftWheel.brakeTorque = 0;
                    axleInfo.rightWheel.brakeTorque = 0;
                    ResetDriftFriction(axleInfo.leftWheel);
                    ResetDriftFriction(axleInfo.rightWheel);
                }

            }
        }

        void ResetDriftFriction(WheelCollider wheel)
        {
            AxleInfo axleInfo = axleInfos.FirstOrDefault(axle => axle.leftWheel == wheel || axle.rightWheel == wheel);
            if (axleInfo == null) return; //Bails out

            wheel.forwardFriction = axleInfo.originalForwardFriction;
            wheel.sidewaysFriction = axleInfo.originalSidewaysFriction;
        }

        void ApplyDriftFriction(WheelCollider wheel)
        {
            if(wheel.GetGroundHit(out var hit))
            {
                wheel.forwardFriction = UpdateFriction(wheel.forwardFriction);
                wheel.sidewaysFriction = UpdateFriction(wheel.sidewaysFriction);
                isGrounded = true;
            }
        }

        private WheelFrictionCurve UpdateFriction(WheelFrictionCurve friction)
        {
            friction.stiffness = input.IsBraking ? Mathf.SmoothDamp(friction.stiffness, .5f, ref driftVelocity, Time.deltaTime * 2f) : 1f;
            return friction;
        }


        //Normalises the input so it SHOULD stay smooth
        float AdjustInput(float input)
        {
            return input switch
            {
                >=  .7f => 1f,
                <= -.7f => -1f,
                _ => input
            };
        }
    }
}

        
        

    