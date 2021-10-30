from unittest import TestCase

from reactor.geometry.vector import Vector2
from ..configspace import ConfigLine


class ConfigLineTestCase(TestCase):

    def setUp(self):
        super().setUp()
        self.p1 = Vector2(0, 0)
        self.p2 = Vector2(1, 1)
        self.config_line = ConfigLine(self.p1, self.p2)

    def test_randomly_sample_config_line(self):
        result = self.config_line.randomly_sample_config_line()
        self.assertTrue((result >= self.p1).all())
        self.assertTrue((result <= self.p2).all())

    def test_randomly_sample_config_line_discrete(self):
        result = self.config_line.randomly_sample_config_line_discrete()
        self.assertTrue((result == self.p1).all() or (result == self.p2).all())

    def test_config_lines_length(self):
        result = self.config_line.config_lines_length()
        self.assertEqual(1.4142135623730951, result)

    def test_config_lines_sq_length(self):
        result = self.config_line.config_lines_sq_length()
        self.assertEqual(2.0000000000000004, result)

    def test_translate_config_line(self):
        self.config_line.translate_config_line(Vector2(1, 1))
        self.assertTrue((Vector2(1, 1) == self.config_line.pos1).all())
        self.assertTrue((Vector2(2, 2) == self.config_line.pos2).all())
