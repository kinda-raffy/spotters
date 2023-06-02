# Spotters

## Guidelines

- Do not push to main.
- Categorise branches via `/` (`nav/localisation`, `nav/init_graph`)
- Submodules have been defaulted to `dev` in their respective repositories. To update dependencies in `spotters`, `git submodule update --remote` will replace the current commit with the latest on `dev`.
- To work on submodules, create an approapiate branch and merge with `dev` to publish changes. 

## Connect to SensorRelay

Open the `Spot SensorRelay` on your iPhone device.

On Ubuntu,

```bash
sudo modprobe v4l2loopback      # Create dummy virtual webcam.
v4l2-ctl --list-devices         # Find the /dev/video[number] of the dummy camera.
# Define pipeline to decode then stream into the dummy device.
gst-launch-1.0 souphttpsrc location=http://[ip]:[port]/rgb ! jpegdec ! videoconvert ! v4l2sink device=/dev/video[number]
```
