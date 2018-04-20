# ROS 
## usb_cam
publish /image_raw 

## vision bridge
transform ros image to opencv mat


## Localization
Maintain the feature map and the trajectory of the camear

subscribe image and callback feature detection and description

given feature map and features from new image, calculate position

record and publish the position

### Feature map
how to maintain the feature map (vector<> & counter and history time)
update


### Feature match
how to match features (flann)

### calculate position
optimization based method (reprojection)

### feature detection and description module (should be easily changed)
received a mat and return a list of key points with descriptor (surf)





## checker
sync the position reported by robot arm and by camera, output result or calculate accuracy.




