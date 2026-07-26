# Orbbec Gemini 435Le network and ROS 2 bring-up

State verified on 2026-07-26.

## Symptom

OrbbecViewer discovered the connected Gemini 435Le but could not open its
streams. The camera did not appear in `lsusb` or as `/dev/video*`.

The Gemini 435Le is an Ethernet/PoE camera, so USB enumeration and Linux UVC
device nodes are not expected.

## Root cause

The camera and host were on different IPv4 subnets:

```text
Gemini 435Le       192.168.1.10/24  MAC 54:14:fd:24:68:68
host enp6s0        172.16.0.6/24
left FR3           172.16.0.2
right FR3          172.16.0.3
```

Ethernet broadcast discovery could identify the camera, but the SDK could not
establish its TCP control and RTSP stream connections. An earlier Viewer
session had temporarily added `192.168.1.53` to the host and used a volatile
Force IP operation, which is why the camera had worked only during that
session.

## Permanent host configuration

`Wired connection 1` now persistently assigns both subnets to `enp6s0`:

```text
172.16.0.6/24
192.168.1.53/24
```

The change was applied without deactivating or reconnecting the interface:

```bash
nmcli connection modify 'Wired connection 1' \
  ipv4.addresses '172.16.0.6/24,192.168.1.53/24' \
  ipv4.method manual
nmcli device modify enp6s0 +ipv4.addresses 192.168.1.53/24
```

`nmcli device modify` reapplies the address configuration to the active
device without intentionally bouncing the Ethernet link. During this change,
simultaneous probes to both FR3 controllers each received 25 of 25 replies
with zero packet loss.

Verify the persistent profile, live addresses, and camera control endpoint:

```bash
nmcli -g ipv4.addresses connection show 'Wired connection 1'
ip -4 -br address show dev enp6s0
nc -vz -w 3 192.168.1.10 8090
```

Expected output includes both host addresses and a successful TCP connection
to port 8090.

## Stream verification

After restarting OrbbecViewer, SDK 2.9.3:

- created a pipeline for Gemini 435Le serial `CP4E46300048`;
- negotiated the color RTSP session from `192.168.1.10:8888` to
  `192.168.1.53`;
- entered `STREAMING@Color`;
- received color frames at approximately 10.1 FPS.

This verifies actual image reception, not only discovery or TCP reachability.
The SDK log is:

```text
/opt/OrbbecSDK_v2.9.3/tools/Log/OrbbecSDK.20260726142349.log.txt
```

Do not run OrbbecViewer and the ROS 2 camera node simultaneously. Close one
client before opening the other.

Use the SDK v2 Viewer installed with the Gemini 435Le tools:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
./scripts/start_orbbec_viewer.sh
```

Do not use
`~/Downloads/OrbbecViewer_v1.10.35_202601290127_linux_x64_release/OrbbecViewer`.
That executable belongs to the SDK v1 Viewer and does not provide the working
Gemini 435Le Ethernet path. The launcher above selects
`/opt/OrbbecSDK_v2.9.3/tools/OrbbecViewer`, supplies the desktop X11
environment, and refuses to start if the ROS camera service or another Viewer
already owns the device.

## ROS 2 service

The Docker image pins `OrbbecSDK_ROS2` v2.8.6, the SDK v2 wrapper required by
the Gemini 435Le. Its bundled x64 SDK is network-only, but the upstream node
still unconditionally configures a USB/UVC backend during initialization.
The image applies
`docker/patches/orbbec_ros2_skip_uvc_for_network.patch` so an explicit network
endpoint skips that irrelevant UVC call. Without the patch, startup fails
with `Usb pal is not exist`.

The Compose service uses the explicit endpoint rather than network
enumeration:

```yaml
orbbec:
  command:
    - ros2
    - launch
    - orbbec_camera
    - gemini435_le.launch.py
    - enumerate_net_device:=false
    - net_device_ip:=192.168.1.10
    - net_device_port:=8090
    - connection_delay:=1
    - enable_point_cloud:=false
    - enable_colored_point_cloud:=false
```

Start and inspect only the camera:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up -d orbbec
docker compose logs -f orbbec
```

The service is part of the default workcell, so an ordinary
`docker compose up` starts it together with the robot pipeline. Explicitly
targeting `orbbec` as above remains useful for camera-only diagnostics.

Primary expected topics:

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
/camera/depth/camera_info
```

Confirm actual messages rather than relying only on topic discovery:

```bash
docker compose exec orbbec timeout 15 \
  ros2 topic hz /camera/color/image_raw
docker compose exec orbbec timeout 15 \
  ros2 topic hz /camera/depth/image_raw
```

The service was rebuilt and exercised against the physical camera on
2026-07-26. The node identified serial `CP4E46300048`, reported an Ethernet
connection to `192.168.1.10:8090`, and published real messages on both raw
image topics. With each stream active, `/camera/device_status` reported:

```text
color_frame_rate_avg:  9.991607832975063
depth_frame_rate_avg: 10.000080002800043
device_online: true
connection_type: Ethernet
```

The configured stream rate is 10 FPS. The Python implementation behind
`ros2 topic hz` reported only approximately 2-4 FPS while deserializing the
1280x800 raw images. Do not interpret that CLI result as the camera source
rate: the C++ driver's counters above showed that the source continued at
10 FPS during the same measurement. For a quick source-rate check, keep an
image subscriber active and inspect `/camera/device_status`.

## Franka communication isolation

Adding a second IP address does not itself generate network traffic. Camera
streaming does. RGB-D traffic on the same physical NIC can contend for receive
queues, interrupt handling, switch buffers, and CPU time with the 1 kHz FCI
sessions. A successful ping test is evidence against a link interruption, but
it is not proof that worst-case FCI jitter is unchanged under sustained
camera load.

The following non-FCI checks passed while ROS image streams were active:

- color and depth source streams each sustained 10 FPS;
- both `172.16.0.2` and `172.16.0.3` returned 500/500 probes during the
  stream tests;
- average RTT was 0.068-0.079 ms;
- `enp6s0` reported zero RX/TX errors, drops, and missed packets.

No arm controller was started for this test. These results show that the
camera did not interrupt ordinary IP communication, but they do not measure
the deadline/jitter behavior of libfranka's 1 kHz FCI loop.

For production use, keep the FR3 traffic physically isolated:

```text
enp6s0                 172.16.0.6/24       FR3 network only
dedicated GigE NIC     192.168.1.53/24     Orbbec only
```

A USB 3-to-Gigabit Ethernet adapter is sufficient if it has a stable Linux
driver and its own USB host bandwidth. Connect the camera/PoE injector or its
camera-only switch to that adapter. Move `192.168.1.53/24` from `enp6s0` to
the dedicated interface, then verify that the route selects that interface:

```bash
ip route get 192.168.1.10
```

Until physical isolation is installed, do not treat camera streaming as
hardware-qualified alongside active FCI merely because Viewer or ROS topics
work. Validate the complete workload while monitoring Franka communication
errors and NIC packet drops before data collection.

The ROS driver starts streams on subscriber demand. Leaving the `orbbec`
service running without image subscribers does not continuously send the
full RGB-D payload; `/camera/device_status` reports zero active frame rate in
that state.

## Rollback

To remove only the camera subnet while keeping the FR3 address:

```bash
nmcli connection modify 'Wired connection 1' \
  ipv4.addresses '172.16.0.6/24'
nmcli device modify enp6s0 -ipv4.addresses 192.168.1.53/24
```

Do not deactivate `enp6s0` while an FCI session is active.
