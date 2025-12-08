
using System;
using Unity.Netcode;
using UnityEngine;

namespace MetaVoiceChat.NetProviders.NGO
{
	public readonly struct NGOFrame
	{
		public readonly int index;
		public readonly double timestamp;
		public readonly float additionalLatency;
		public readonly ArraySegment<byte> data;

		public readonly ushort Length => (ushort)data.Count;

		public NGOFrame(int index, double timestamp, float additionalLatency, ArraySegment<byte> data)
		{
			this.index = index;
			this.timestamp = timestamp;
			this.additionalLatency = additionalLatency;
			this.data = data;
		}

		public NGOFrame(int index, double timestamp, float additionalLatency)
		{
			this.index = index;
			this.timestamp = timestamp;
			this.additionalLatency = additionalLatency;
			data = ArraySegment<byte>.Empty;
		}
	}

	public static class NGOFrameReaderWriter
	{
		// There is probably a better place to set this limitation
		private const float MaxAdditionalLatency = 0.2f;

		public static unsafe void ReadValueSafe(this FastBufferReader reader, out NGOFrame frame)
		{
			reader.ReadValueSafe<int>(out int index);
			reader.ReadValueSafe<double>(out double timestamp);
			reader.ReadValueSafe<byte>(out byte compressedLatency);
			reader.ReadValueSafe<ushort>(out ushort length);

			float t = (float)compressedLatency / byte.MaxValue;
			float additionalLatency = t * MaxAdditionalLatency;

			if (length > 0)
			{
				var data = new byte[length];
				reader.ReadBytesSafe(ref data, length);
				frame = new NGOFrame(index, timestamp, additionalLatency, data);
			}
			else
			{
				frame = new NGOFrame(index, timestamp, additionalLatency);
			}
		}

		public static void WriteValueSafe(this FastBufferWriter writer, in NGOFrame frame)
		{
			float additionalLatency = frame.additionalLatency;
			additionalLatency = Mathf.Clamp(additionalLatency, 0, MaxAdditionalLatency);
			float t = additionalLatency / MaxAdditionalLatency;

			ushort dataLength = frame.Length;

			writer.WriteValueSafe<int>(frame.index);
			writer.WriteValueSafe<double>(frame.timestamp);
			writer.WriteValueSafe<byte>((byte)(t * byte.MaxValue));
			writer.WriteValueSafe<ushort>(dataLength);

			if (dataLength > 0)
			{
				writer.WriteBytesSafe(frame.data.Array, dataLength, frame.data.Offset);
			}
		}
	}
}
