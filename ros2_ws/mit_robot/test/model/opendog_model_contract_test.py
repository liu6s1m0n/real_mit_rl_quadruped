"""Compare the OpenDog dynamics model with the current DM1 MJCF, without simulation."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
URDF = ET.parse(ROOT / 'config/algorithm/dm1_opendog.urdf').getroot()
MJCF = ET.parse(ROOT / 'src/dm1_description/dm1/dm1.xml').getroot()
LEGS = {'RF': 'FR', 'LF': 'FL', 'RH': 'RR', 'LH': 'RL'}


class OpenDogModelContract(unittest.TestCase):
    def assertVectorEqual(self, left, right, places=10):
        for actual, expected in zip(map(float, left.split()), map(float, right.split())):
            self.assertAlmostEqual(actual, expected, places=places)

    def test_total_mass(self):
        actual = sum(float(x.attrib['value']) for x in URDF.findall('link/inertial/mass'))
        expected = sum(float(x.attrib['mass']) for x in MJCF.findall('.//worldbody//inertial'))
        self.assertAlmostEqual(actual, expected, places=10)

    def test_link_inertias(self):
        links = [('base', 'trunk')]
        links += [(f'{old}_{"FOOT" if part == "foot" else part}', f'{new}_{part}')
                  for old, new in LEGS.items() for part in ('hip', 'thigh', 'calf', 'foot')]
        for name, body in links:
            with self.subTest(link=name):
                actual = URDF.find(f"link[@name='{name}']/inertial")
                expected = MJCF.find(f".//body[@name='{body}']/inertial")
                self.assertAlmostEqual(float(actual.find('mass').attrib['value']),
                                       float(expected.attrib['mass']), places=10)
                self.assertVectorEqual(actual.find('origin').attrib['xyz'], expected.attrib['pos'])
                values = actual.find('inertia').attrib
                diag = ' '.join(values[key] for key in ('ixx', 'iyy', 'izz'))
                self.assertVectorEqual(diag, expected.attrib.get('diaginertia',
                                       ' '.join(expected.attrib.get('fullinertia', '').split()[:3])))
                offdiag = ' '.join(values[key] for key in ('ixy', 'ixz', 'iyz'))
                self.assertVectorEqual(offdiag, ' '.join(expected.attrib['fullinertia'].split()[3:])
                                       if 'fullinertia' in expected.attrib else '0 0 0')

    def test_joint_frames_axes_and_limits(self):
        for old, new in LEGS.items():
            for part, suffix, cls in [('hip', 'HAA', 'haa'), ('thigh', 'HFE', 'hfe'),
                                      ('calf', 'KFE', 'kfe'), ('foot', 'foot_fixed', None)]:
                with self.subTest(leg=new, part=part):
                    joint = URDF.find(f"joint[@name='{old}_{suffix}']")
                    body = MJCF.find(f".//body[@name='{new}_{part}']")
                    self.assertVectorEqual(joint.find('origin').attrib['xyz'], body.attrib['pos'])
                    quat = list(map(float, body.attrib.get('quat', '1 0 0 0').split()))
                    self.assertVectorEqual(joint.find('origin').attrib['rpy'],
                                           f'0 {2 * math.atan2(quat[2], quat[0])} 0')
                    if cls is None:
                        self.assertEqual(joint.attrib['type'], 'fixed')
                        continue
                    default = MJCF.find(f".//default[@class='{cls}']/joint")
                    self.assertVectorEqual(joint.find('axis').attrib['xyz'], default.attrib['axis'])
                    limits = body.find('joint').attrib.get('range', default.attrib.get('range'))
                    self.assertVectorEqual(joint.find('limit').attrib['lower'] + ' ' +
                                           joint.find('limit').attrib['upper'], limits)


if __name__ == '__main__':
    unittest.main()
