# Orbbec cameras

This workcell uses two cameras:

| Camera | Link | ROS namespace |
|--------|------|----------------|
| Gemini 435Le | Ethernet `192.168.35.35:8090` on `enp2s0f3` | `/camera` |
| Gemini 305 | USB Type-C, VID `2bc5` | `/gemini305` |

The 435Le is Ethernet/PoE, so it does not appear in `lsusb` or `/dev/video*`. The 305 is powered and streamed through the same Type-C cable (`DC 5V ≥ 700 mA`, USB 3.0 recommended).

Verified 435Le on this host: serial `CP4N5630008Z`. Verified 305: serial `CV2T661000EV` on USB 3.2. Older notes used 435Le `CP4E46300048` at `192.168.1.10`.

## Network (435Le)

Discovery can work across a misconfigured host while streams still fail. This workstation's camera NIC is `enp2s0f3` (`camera`, `192.168.35.6/16`). Verify without changing a live FCI interface:

```bash
ip -4 -br address show dev enp2s0f3
nc -vz -w 3 192.168.35.35 8090
lsusb | grep -iE '2bc5|orbbec'
```

Never bounce or reconfigure a Franka NIC during an active FCI session.

## USB (305)

```bash
lsusb | grep -iE '2bc5|orbbec'
```

Plug the Type-C cable into a USB 3.0 port. Charge-only cables will not enumerate. If ROS already owns the camera, unplug/replug will not help until `orbbec-305` is stopped.

## ROS

```bash
cd docker
docker compose up -d orbbec orbbec-305
docker compose logs -f orbbec orbbec-305
```

Expected 435Le topics: `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/depth/image_raw`, `/camera/depth/camera_info`.
Expected 305 topics: the same names under `/gemini305`.

The 435Le source rate is 10 FPS; Python `ros2 topic hz` can under-report while deserializing 1280×800 images, so `/camera/device_status` is the authoritative source counter.

The image pins the SDK v2 ROS wrapper and applies `docker/patches/orbbec_ros2_skip_uvc_for_network.patch` only when a network IP is set. The USB 305 launch does not set that IP, so it still initializes libuvc.

## Operator GUI camera tab

The desktop Operator GUI deliberately does not load an Orbbec or ROS driver.
Start the lightweight snapshot bridge beside the camera drivers:

```bash
cd docker
docker compose up -d orbbec orbbec-305 camera-view
```

The bridge subscribes to compressed ROS images and exposes only the newest JPEG
on host port `8091`. The GUI polls at at most 8 FPS with one request in flight,
so a slow remote display cannot queue old frames or block robot control. It
automatically shows an offline placeholder and reconnects when frames return.

The default camera selectors are:

- `Gemini 435Le`: `/camera/color/image_raw/compressed`
- `305 相机`: `/gemini305/color/image_raw/compressed`

## Viewer

Stop both ROS camera services first, then run:

```bash
./ops/run/start_orbbec_viewer.sh
```

Use the SDK v2 viewer selected by the script. Only one Viewer or ROS client may own a given camera.

## FCI isolation

Keep RGB-D traffic on `enp2s0f3`. Confirm `ip route get 192.168.35.35` selects that NIC.
