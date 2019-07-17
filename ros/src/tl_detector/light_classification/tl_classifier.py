from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os
import cv2
import numpy as np
import rospy

IMAGE_WIDTH = 300
IMAGE_HEIGHT = 300

class TLClassifier(object):
    def __init__(self, is_site):
        # tensorflow frozen graph
        self.graph = None

        # tensorflow session
        self.session = None

        # avaliable traffic light classes, corresponding to trained net
        self.classes = {1: TrafficLight.RED, 2: TrafficLight.YELLOW, 3: TrafficLight.GREEN, 4: TrafficLight.UNKNOWN }

        # image counter to process only the n-th image to reduce load
        self.counter = 0

        self.class_index = None
        self.prob = None

        # load fixed simulated model - may change according to simulation or real
        # but as this is an individual submission it will never be used on Carla according to
        # project description --> set fixed
        if is_site:
            self.load_model(os.path.dirname(os.path.realpath(__file__)) + '/../../../../TrafficDetectionNet/workspace/tl_classifier/model_frozen_real/frozen_inference_graph.pb')
        else:
            self.load_model(os.path.dirname(os.path.realpath(__file__)) + '/../../../../TrafficDetectionNet/workspace/tl_classifier/model_frozen_sim/frozen_inference_graph.pb')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # activate logging to get information on GPU usage
        tf.logging.set_verbosity(tf.logging.INFO)

        # only process every 5th frame
        if self.counter == 0:
            self.class_index, self.prob = self.predict(image)
            self.counter = 5        # only use every 5th image to save bandwidth

        self.counter = self.counter - 1

        """
        if self.class_index is not None:
            rospy.logwarn("class %d, prob: %f", self.class_index, self.prob)
        """

        return self.class_index

    def load_model(self, path):
        # load tf model according to description in tf object detection tutorial
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        self.graph = tf.Graph()

        # setup tf session for classification
        with tf.Session(graph=self.graph, config=config) as sess:
            self.session = sess
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def predict(self, numpy_image, thresh=0.5):
        # get tensor for input image
        image_tensor = self.graph.get_tensor_by_name('image_tensor:0')

        # get tensors for output data (detected boxes, scores and classes)
        detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.graph.get_tensor_by_name('detection_classes:0')

        # preprocess the image (scaling and format) to be used by the classification net
        numpy_image = self.preprocess_image(numpy_image)

        # run prediction for current image
        (boxes, scores, classes) = self.session.run([detection_boxes, detection_scores, detection_classes],
            feed_dict={image_tensor: np.expand_dims(numpy_image, axis=0)})

        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        boxes = np.squeeze(boxes)

        # check for a box with confidence above given threshold and return light class
        for i, box in enumerate(boxes):
            if scores[i] > thresh:
                light_class = self.classes[classes[i]]
                rospy.logdebug("Traffic light detectd %d", light_class)
                return light_class, scores[i]

        return None, None

    def preprocess_image(self, image):
        image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image
