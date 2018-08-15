
import unittest
import pyhhi.build.common.ver as ver


class VersionTestCase(unittest.TestCase):

    def test_ubuntu_version_tuple_to_str(self):
        self.assertEqual("12.04", ver.ubuntu_version_tuple_to_str((12,4)))

    def test_version_list_sort(self):
        self.assertEqual([(4,7), (4,8,0), (4,8,1)], ver.version_list_sort([(4,8,1), (4,7), (4,8,0)]))
        self.assertEqual([(4,8,0), (4,8,1), (4,9)], ver.version_list_sort([(4,8,1), (4,9), (4,8,0)]))

    def test_version_tuple_from_str(self):
        self.assertEqual((0, 1, 2, 3), ver.version_tuple_from_str('0.1.2-3beta-2'))
        self.assertEqual(tuple([10]), ver.version_tuple_from_str('10'))
        self.assertEqual(tuple([10]), ver.version_tuple_from_str('10rc2'))
        self.assertEqual(tuple([10]), ver.version_tuple_from_str('10-rc2'))
        self.assertEqual((10, 7, 4), ver.version_tuple_from_str('10.7.4'))
        self.assertEqual((10, 7, 4), ver.version_tuple_from_str('10.7-4'))
        self.assertEqual((10, 7, 4, 0), ver.version_tuple_from_str('10.7-4', 4))
        self.assertEqual((10, 7, 8, 9), ver.version_tuple_from_str('10.7.8.9-4, 4', 4))
        #
        self.assertEqual(('1.2', '3'), ver.version_str_to_rpm_version_tuple('1.2-3'))
        self.assertEqual(('1.2', '3'), ver.version_str_to_rpm_version_tuple('1.2.3'))

    def test_version_tuple_to_str(self):
        self.assertEqual('10', ver.version_tuple_to_str(tuple([10])))
        self.assertEqual('10.2', ver.version_tuple_to_str((10, 2)))

    def test_boost_version_str(self):
        self.assertEqual('1_56_0', ver.get_boost_version_str('1.56.0'))

    def test_version_compare(self):
        self.assertEqual(0, ver.version_compare(ver.version_tuple_from_str('10.7.4'), ver.version_tuple_from_str('10.7.4')))
        self.assertEqual(1, ver.version_compare(ver.version_tuple_from_str('10.7.4'), ver.version_tuple_from_str('10.6.4')))
        self.assertEqual(-1, ver.version_compare(ver.version_tuple_from_str('10.6.4'), ver.version_tuple_from_str('10.7.4')))
        self.assertEqual(-1, ver.version_compare(ver.version_tuple_from_str('10.7'), ver.version_tuple_from_str('10.7.4')))
        self.assertEqual(1, ver.version_compare(ver.version_tuple_from_str('10.7.4'), ver.version_tuple_from_str('10.7')))
        self.assertEqual(0, ver.version_compare(ver.version_tuple_from_str('10.7.0'), ver.version_tuple_from_str('10.7')))
        
