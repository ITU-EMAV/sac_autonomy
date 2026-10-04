# Runtime models

Model binaries are distributed in the `runtime-assets-v1` Release of
`ITU-EMAV/sac_autonomy`. Install the pinned release with:

```bash
./utilities/tools/download_assets.sh
```

The archive contains YOLOPv2, traffic-sign, object and road-hazard weights/engines,
plus maps and geographic lookup data. Downloads and files are checked with SHA-256.
TensorRT engines originate from the Jetson environment; rebuild them for a different
GPU or TensorRT version using the supplied model export scripts.
