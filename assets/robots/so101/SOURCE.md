# SO-101 model source

Generated from the Apache-2.0 licensed `so101_follower.urdf` and STL meshes in [legalaspro/so101-ros-physical-ai](https://github.com/legalaspro/so101-ros-physical-ai).

Source commit: `58318c905a2c61289fa907de85cb8473322fbe68`

## Wrist-camera attachment

No wrist-camera attachment mesh is included or rendered. Several published and
photo-derived candidates were compared with the installed solid white bracket,
but none matched its geometry. Calibration therefore treats that attachment as
unmapped occluding geometry and excludes its region from stock-mesh fitting.
