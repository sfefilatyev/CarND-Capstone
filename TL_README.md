## Traffic Light Classification (TLC) with Single Shot Multibox Detector (SSD)

_Disclaimer: Due to limited time and GPU resources, we borrowed a fully trained model from another team. We have also learned how to 
train a model using [Object Detection Model Zoo](https://github.com/tensorflow/models/blob/r1.5/research/object_detection/g3doc/detection_model_zoo.md)
from the same group's [tutorial](https://github.com/alex-lechner/Traffic-Light-Classification/blob/master/README.md), 
for which we are very grateful._

### TLC Intro

The goal of TLC was to train a model for quick classification of the traffic lights in the simulator environment and in
Carla autonomous driving vehicle in the real Udacity parking lot.

### TLC Implementation

After a quick Google search and reviewing the Object Detection Lab in the Udacity lectures, we realized that 
we would not have to train our model from scratch.  There's already a large library of object detection models hosted by 
Tensorflow and contributed by Google [link](https://github.com/tensorflow/models/tree/r1.5/research/object_detection).
We chose SSD model based on the report by Alex Lechner group that this was the optimal model for them.

**Figure:  SSD Model** 
![SSD Model](imgs/SSD_model.jpg)

SSD is end-to-end model [ref](https://arxiv.org/pdf/1512.02325.pdf), meaning it is trained to detect and classify an object in the picture
at the same time.  SSD is a CNN built on the base of VGG16. It accepts images of any size and uses kernels to scan them for features at different
levels of detail.  SSD gradually shrinks the feature map size and increase the depth as it goes to the deeper layers. 
The deep layers cover larger receptive fields and construct more abstract representation, while the shallow layers cover 
smaller receptive fields.  SSD uses shallow layers to predict small/further objects and deeper layers to predict big objects.
As can be seen in the diagram above, the result of these predictions at different levels are a bunch of bounding boxes.
The boxes are combined around the object into a single bounding box which is output together with the object classification.

**Figure:  SSD Feature Layer Boxes** 
![SSD Model](imgs/SSD_model_boxes.jpg)

We decided to adopt another group's ([model](https://github.com/alex-lechner/Traffic-Light-Classification/tree/master/models)) 
just to try the whole pipeline together.  As we found out, that model turned out to be too slow in a simulator working on CPU.
It took 1-1.5 second to classify a single image posted to _/image_color_ topic.  Because the Udacity skeleton code was calling the TLClassifier 
for every image, TLDetector thread was lagging behind trying to post every light detection to _/traffic_waypoint_ topic.
To deal with this issue, we introduced 
Looking at the [Model Zoo]((https://github.com/tensorflow/models/blob/r1.5/research/object_detection/g3doc/detection_model_zoo.md))


Due to the lack of time, we skipped the data generation and labeling by borrowing the group's TFRecord files 
[link](https://github.com/alex-lechner/Traffic-Light-Classification/blob/master/README.md#1-the-lazy-approach).

