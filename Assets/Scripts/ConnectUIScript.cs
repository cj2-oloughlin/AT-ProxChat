using Unity.Netcode;
using UnityEngine;
using UnityEngine.UI;

public class ConnectUIScript : MonoBehaviour
{
    [SerializeField] private Button hostButton;
    [SerializeField] private Button clientButton;


    private void Start()
    {
        hostButton.onClick.AddListener(HostButtonOnClick);
        hostButton.onClick.AddListener(ClientButtonOnClick);
    }


    private void HostButtonOnClick()
    {
        NetworkManager.Singleton.StartHost();
    }

    private void ClientButtonOnClick()
    {
        NetworkManager.Singleton.StartClient();
    }
}
