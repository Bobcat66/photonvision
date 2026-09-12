# ML-Accelerated Apriltag Detection

Photonvision can use a (comparatively cheap) object detection model to filter the image before processing. This can result in performance improvements as the (more expensive) apriltag detector will have to search a smaller area.

:::{note}
ML Apriltag acceleration is currently only supported on the Rubik Pi and the Orange Pi 5
:::
