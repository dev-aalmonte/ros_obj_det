# ROS2 Object Detection
### NOTE: (The project only works with the specific hardware I used, it is not generic enough to use it as a standalone node. which means it might not work if you try to use it as is)
---
Practice to Train and detect custom models on Yolov5 and publish Image message topic and TF on ROS2.

## Build
1. Clone the repository
> cd {ROS2 workspace path}/src
> git clone https://github.com/dev-aalmonte/ros_obj_det.git
2. Build and source
> colcon build
> source install/setup.bash
3. Launch
> ros2 launch ros_obj_det ros_obj_det.launch.py

## Approach
To train the data I use Yolov5 and followed the training process published in their website "[Train Custom Data](https://github.com/ultralytics/yolov5/wiki/Train-Custom-Data)". It consisted in 3 steps: labeling, training and detection.

### Labeling
For the labeling I use the website [Roboflow](https://app.roboflow.com/) to label the data. They can also train the data for you and it has multiple features like private workspace, however those features are only available for premium accounts. Since I only need it to label my images, the free version is more than enough.

Some labeling mention is also [LabelStudio](https://labelstud.io/) which can only do labeling, however is open source and can run locally on your machine, no internet connection required.

### Training
For training I use Yolov5, which is an older version of the training model, you can use the most recent version if needed. For the installation I follow the following instructions (Recommended: clone the repo outside of your workspace),
> git clone https://github.com/ultralytics/yolov5  
> cd yolov5
> pip install -r requirements.txt 

To do the training portion you need to execute this command
> python train.py --img 640 --batch 16 --epochs 3 --data coco128.yaml --weights yolov5s.pt

Where:
- **img**: Image Size
- **batch**: Batch Size (Number has to be power of 2)
- **epochs**: Epoch Amount (The bigger the better and the longer it takes to execute)
- **data**: The .yaml of your trained data
- **weight**: The yolo model used to train the data

This process can take a lot of time depending on how much data you decided to train and the resolution of the image.

Once your training is over it will be stored inside the **runs** folder in the yolov5 project. It gives you the best and the last run in .pt format (Recommended: Use always the best).

To export the .pt file to your preferred format you use the following command in Yolov5

> python export.py --weights best.pt --include onnx

Where:
- **weights**: Path to the .pt file
- **include**: format to export

This command will create a .onnx file in the same path as the .pt file and can be used to detect objects.

### Detection
This step it varies depending on the hardware used to capture the image. In my case, I'm using the [Zed2i AI Camera](https://www.stereolabs.com/products/zed-2) from [StereoLabs](https://www.stereolabs.com/). They have a SDK and more important for the project, a ROS2 wrapper where they publish topics regarding the **camera image**, **depth image** and **point cloud**.

The steps I took to do the implementation was:
1. Load the model
> self.model = torch.hub.load("ultralytics/yolov5", "custom", f"{package_share_path}/models/best.onnx", verbose=False)
2. Grab frame from the camera
On that step I subscribed to the **/zed/zed_node/left_raw/image_raw_color** topic and from the callback I converted the image msg to a OpenCV Image using the CVBridge
>  cv_img : cv2.Mat = self.bridge.imgmsg_to_cv2(msg)
3. Send image to the model
It gives me a result, where it is the **min_x**, **min_y**, **max_x**, **max_y** coordinates on where the object is detected, as well as some extra information like: **class**, **name** and **confidence**
> result = self.model(resized)
> pandas = result.pandas().xyxy[0]
4. Get center point of the object
This one is easy enough, it just to take the difference of the **min** and **max** coordinates, that divide it by 2.
> center = (
>    int(max_orig[0] - ((max_orig[0] - min_orig[0]) / 2)),
>    int(max_orig[1] - ((max_orig[1] - min_orig[1]) / 2))
> )
5. Get the Point cloud representation of the 2D Point
You need to create a subscriber that listen for the point cloud, in my case is **/zed/zed_node/point_cloud/cloud_registered**. With that information you need to get the exact point on the point cloud that represent the 2D Point, which can be done by doing the following calculation.
>point = w * msg.point_step + h * msg.row_step
6. Convert the point cloud data from **Int8** to **Float32**
The data is given an **Int8** array where each point is offset by 4 position:
**x = point**  
**y = point + 4** 
**z = point + 8** 
There is a field in the message that tells you the offset of each point if the data structure changes. There is also a value in the message that tells you which endian is (This is important in order to make the conversion)
The final code looks something like this
> x_pos = point + msg.fields[0].offset
> y_pos = point + msg.fields[1].offset
> z_pos = point + msg.fields[2].offset
>
> x = (msg.data[x_pos + 3] << 8 * 3) + (msg.data[x_pos + 2] << 8 * 2) + (msg.data[x_pos + 1] << 8) + msg.data[x_pos]
> y = (msg.data[y_pos + 3] << 8 * 3) + (msg.data[y_pos + 2] << 8 * 2) + (msg.data[y_pos + 1] << 8) + msg.data[y_pos]
> z = (msg.data[z_pos + 3] << 8 * 3) + (msg.data[z_pos + 2] << 8 * 2) + (msg.data[z_pos + 1] << 8) + msg.data[z_pos]
>
> endian = "<>"[msg.is_bigendian]
>
> x_float = struct.unpack(f'{endian}f', struct.pack('I', x))
> y_float = struct.unpack(f'{endian}f', struct.pack('I', y))
> z_float = struct.unpack(f'{endian}f', struct.pack('I', z))

You can review the **yolo_obj_det.py** file to check the full ROS2 implementation.