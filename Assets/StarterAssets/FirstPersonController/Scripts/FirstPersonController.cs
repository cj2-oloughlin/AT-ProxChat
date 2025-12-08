using Unity.Netcode;
using UnityEngine;
using UnityEngine.InputSystem;
using System;
using System.Collections;

public class FirstPersonController : NetworkBehaviour
{
    private CharacterController _controller;
    private Camera _camera;
    private Animator _animator;

    private float TurnSmoothVelocity;
    [SerializeField] private float turnSmoothTime = 0.1f;
    [SerializeField] private float speed = 5f;

    public override void OnNetworkSpawn()
    {
        _controller = GetComponent<CharacterController>();
        _camera = GetComponentInChildren<Camera>();
        _animator = GetComponentInChildren<Animator>();

        Cursor.visible = false;
        Cursor.lockState = CursorLockMode.Locked;

        if (!IsOwner)
        {
            _camera.enabled = false;
            //GetComponentInChildren<CinemachineBrain>().enabled = false;
        }
    }

    void Update()
    {
        if (!IsOwner)
        {
            return;
        }

        Vector2 move2d = new Vector2(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical")).normalized;
        Vector3 move = new Vector3(move2d.x, 0f, move2d.y);

        if (move2d.magnitude >= 0.1F)
        {
            var TargetAngle = Mathf.Atan2(move.x, move.z) * Mathf.Rad2Deg + _camera.transform.eulerAngles.y;
            var angle = Mathf.SmoothDampAngle(transform.eulerAngles.y, TargetAngle, ref TurnSmoothVelocity, turnSmoothTime);
            transform.rotation = Quaternion.Euler(0, angle, 0);

            Vector3 moveDir = Quaternion.Euler(0, TargetAngle, 0) * Vector3.forward;
            _controller.Move(Time.deltaTime * speed * moveDir.normalized);
        }

        if (_controller.velocity.magnitude > 1f)
        {
            // _animator.SetBool ("Insert name of animation, true);
        }

        else
        {
            // _animator.SetBool ("Insert name of animation, false);
        }

        _controller.Move(Time.deltaTime * 5 * new Vector3(0, -0.1f, 0));
    }
}