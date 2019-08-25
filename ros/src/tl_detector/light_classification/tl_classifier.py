from styx_msgs.msg import TrafficLight
import rospy
import tf
import numpy as np
import time

class TLClassifier(object):
    def __init__(self, tl_model_dir, min_detect_score_thresh=0.5):
        # load classifier
        rospy.loginfo('>>> TLClassifier is using model dir: %s, min TL detection threshold: %s', tl_model_dir, str(min_detect_score_thresh))

        # load NN graph
        self.detection_graph = self.load_graph(tl_model_dir + '/frozen_inference_graph.pb')
        self.min_detect_thresh = min_detect_score_thresh

        self.categoty_to_tl_map = {'1': TrafficLight.GREEN, '2': TrafficLight.RED, '3': TrafficLight.YELLOW, '4': TrafficLight.UNKNOWN}
        self.categoty_to_str_map = {'1': 'GREEN', '2': 'RED', '3': 'YELLOW', '4': 'UNKNOWN'}

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # implement light color prediction
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                detect_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                detect_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detect_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                image_np = self.load_image_into_numpy_array(image)
                image_expanded = np.expand_dims(image_np, axis=0)

                t0 = time.time()
                (boxes, scores, classes, num) = sess.run(
                    [detect_boxes, detect_scores, detect_classes, num_detections],
                    feed_dict={image_tensor: image_expanded})
                t1 = time.time()

                traff_light = TrafficLight.UNKNOWN
                tl_col_str = self.categoty_to_str_map[4]
                if scores is not None and len(scores[0]) > 0:
                    highest_score = np.squeeze(scores)[0]
                    if highest_score > self.min_detect_thresh:
                        # A positive detection happened
                        cat_idx_str = str(np.squeeze(classes).astype(np.int32)[0])
                        tl_col_str = self.categoty_to_str_map[cat_idx_str]
                        traff_light = self.categoty_to_tl_map[cat_idx_str]

                rospy.logdebug('----------------------')

                rospy.loginfo('TLClassifier detected %s (%s) light after %d msec', tl_col_str, str(traff_light), (t1 - t0) * 1000)

                rospy.logdebug('TLClassifier SCORES %s', str(scores[0]))
                rospy.logdebug('TLClassifier CLASSES %s', str(classes[0]))
                rospy.logdebug('----------------------')

        return traff_light

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)
