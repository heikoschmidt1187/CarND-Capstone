from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os

IMAGE_WIDTH = 300
IMAGE_HEIGHT = 300

class TLClassifier(object):
    def __init__(self):
        self.graph = None
        self.session = None
        self.image_ctr = 0
        self.classes = {1: TrafficLight.RED, 2: TrafficLight.YELLOW, 3: TrafficLight.GREEN, 4: TrafficLight.UNKNOWN }

        self.load_model(os.path.dirname(os.path.realpath(__file__)) + '/../../../../TrafficDetectionNet/workspace/tl_classifier/model_frozen_sim/frozen_inference_graph.pb')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        class_index, prob = self.predict(image)

        if class_index is not None:
            rospy.logdebug("class %d, prob: %f", class_index, prob)

        return class_index


    def load_model(self, path):
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        self.graph = tf.Graph()

        with tf.Session(graph=self.graph, config=config) as sess:
            self.session = sess
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def predict(self, numpy_image, thresh=0.5):
        image_tensor = self.model_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.model_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.model_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.model_graph.get_tensor_by_name('detection_classes:0')
        numpy_image = self.preprocess_image(numpy_image)

        (boxes, scores, classes) = self.session.run([detection_boxes, detection_scores, detection_classes],
            feed_dict={image_tensor: np.expand_dims(numpy_image, axis=0)})

        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        boxes = np.squeeze(boxes)

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
