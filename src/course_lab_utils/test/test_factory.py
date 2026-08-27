import json
import unittest

from std_msgs.msg import Header

from course_lab_utils.factory_bottle_detector import boxes_to_detection_array
from course_lab_utils.factory_label_reader import label_matches
from course_lab_utils.factory_pipeline import parse_recipe_components
from course_lab_utils.factory_recipe_validator import parse_validation_response


class FactoryHelpersTest(unittest.TestCase):
    def test_parses_validation_json_inside_markdown(self):
        result = parse_validation_response(
            '```json\n{"is_valid": true, "feedback": "ok"}\n```'
        )
        self.assertTrue(result["is_valid"])

    def test_label_match_is_case_insensitive(self):
        self.assertTrue(label_matches("hcl", "Bottle HCl 0.1 mol/L"))

    def test_converts_mock_box_to_vision_message(self):
        detections = boxes_to_detection_array([(10, 20, 50, 100, 0.95)], Header())
        self.assertEqual(len(detections.detections), 1)
        detection = detections.detections[0]
        self.assertEqual(detection.bbox.center.position.x, 30.0)
        self.assertEqual(detection.bbox.size_y, 80.0)
        self.assertEqual(detection.results[0].hypothesis.class_id, "bottle")

    def test_parses_recipe_components(self):
        recipe = json.dumps(
            {"components": [{"name": "Water", "volume_ml": 10}]}
        )
        self.assertEqual(
            parse_recipe_components(recipe),
            [{"name": "Water", "volume_ml": 10.0}],
        )
