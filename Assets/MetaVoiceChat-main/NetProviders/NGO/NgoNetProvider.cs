
using MetaVoiceChat.Utils;
using System;
using System.Collections.Generic;
using Unity.Netcode;
using UnityEngine;

namespace MetaVoiceChat.NetProviders.NGO
{
	[RequireComponent(typeof(MetaVc))]
	public class NGONetProvider : NetworkBehaviour, INetProvider
	{
		public static NGONetProvider LocalPlayerInstance { get; private set; }
		private readonly static List<NGONetProvider> s_Instances = new();
		public static IReadOnlyList<NGONetProvider> Instances => s_Instances;

		public MetaVc MetaVc { get; private set; }

		/* =========================================================================================
		 *									NetworkBehaviour
		 =========================================================================================== */
		public override void OnNetworkSpawn()
		{
			if (IsOwner)
			{
				LocalPlayerInstance = this;
			}
			s_Instances.Add(this);

			static int GetMaxDataBytesPerPacket(NetworkManager netManager)
			{
				const int networkBatchHeaderSize = sizeof(ushort)   // Magic
												   + sizeof(ushort) // BatchCount
												   + sizeof(int)    // BatchSize
												   + sizeof(ulong); // BatchHash

				const int networkMessageHeaderSize = sizeof(uint) * 2; // MessageType, MessageSize
				const int messageNameHashSize = sizeof(ulong);  // MessageName hash

				// Not sure about this overhead but it's essentially the same thing
				// used internally for "Named" and "Unnamed" messages.
				int bytes = netManager.MaximumTransmissionUnitSize
							- networkMessageHeaderSize
							- networkBatchHeaderSize
							- messageNameHashSize;

				bytes -= sizeof(int);    // Index
				bytes -= sizeof(double); // Timestamp
				bytes -= sizeof(byte);   // Additional latency
				bytes -= sizeof(ushort); // Array length
				return bytes;
			}

			MetaVc = GetComponent<MetaVc>();
			MetaVc.StartClient(this, IsOwner, GetMaxDataBytesPerPacket(NetworkManager));
		}

		public override void OnNetworkDespawn()
		{
			if (IsOwner)
			{
				LocalPlayerInstance = null;
			}
			s_Instances.Remove(this);

			MetaVc.StopClient();
		}

		/* =========================================================================================
		 *									INetProvider
		 =========================================================================================== */
		public bool IsLocalPlayerDeafened
		{
			get
			{
				// We cannot speak yet
				if (LocalPlayerInstance == null)
					return false;

				return LocalPlayerInstance.MetaVc.isDeafened;
			}
		}

		public void RelayFrame(int index, double timestamp, ReadOnlySpan<byte> data)
		{
			var array = FixedLengthArrayPool<byte>.Rent(data.Length);
			data.CopyTo(array);

			float additionalLatency = Time.deltaTime;
			NGOFrame frame = new(index, timestamp, additionalLatency, array);

			if (IsServer)
			{
				ReceiveFrameClientRpc(frame);
			}
			else
			{
				RelayFrameServerRpc(frame);
			}

			FixedLengthArrayPool<byte>.Return(array);
		}

		/* =========================================================================================
		 *										RPCs
		 =========================================================================================== */
		[Rpc(SendTo.Server, Delivery = RpcDelivery.Unreliable, RequireOwnership = false)]
		private void RelayFrameServerRpc(NGOFrame frame, RpcParams rpcParams = default)
		{
			float additionalLatency = frame.additionalLatency + Time.deltaTime;
			frame = new(frame.index, frame.timestamp, additionalLatency, frame.data);
			ReceiveFrameClientRpc(frame);
		}

		[Rpc(SendTo.NotOwner, Delivery = RpcDelivery.Unreliable, RequireOwnership = false)]
		private void ReceiveFrameClientRpc(NGOFrame frame, RpcParams rpcParams = default)
		{
			float latency = frame.additionalLatency;
			if (IsServer)
			{
				// Don't apply server Time.deltaTime to additionalLatency -- this frame did not go over the network again.
				latency -= Time.deltaTime;
			}

			MetaVc.ReceiveFrame(frame.index, frame.timestamp, latency, frame.data);
		}
	}
}
