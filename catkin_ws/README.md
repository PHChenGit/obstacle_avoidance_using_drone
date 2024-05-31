## Step 1

## Step 2

## Step 3

## Step 4. Launch rtabmap
```
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
    depth_topic:=/iris/camera/depth/image_raw \
    rgb_topic:=/iris/camera/rgb/image_raw \
    camera_info_topic:=/iris/camera/rgb/camera_info \
    approx_sync:=false \
    rtabmapviz:=true
```