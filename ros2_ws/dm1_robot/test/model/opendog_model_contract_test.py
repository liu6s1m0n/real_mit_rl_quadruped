"""
不运行仿真，直接比较 OpenDog 动力学模型（URDF）与当前 DM1 MJCF 是否一致.

两份模型分别用于 MPC/WBC 和 MuJoCo 仿真，质量、惯量、关节位置/轴/限位必须相同，
否则仿真结果不能代表控制器所假设的模型。
"""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
URDF = ET.parse(ROOT / 'config/opendog/dm1_opendog.urdf').getroot()
MJCF = ET.parse(ROOT / 'description/dm1.xml').getroot()
# OpenDog URDF 腿名 -> DM1 MJCF 腿名。
LEGS = {'RF': 'FR', 'LF': 'FL', 'RH': 'RR', 'LH': 'RL'}


class OpenDogModelContract(unittest.TestCase):
    """URDF 与 MJCF 的模型一致性检查."""

    def assertVectorEqual(self, left, right, places=10):
        """逐元素比较两个以空格分隔的数值字符串."""
        for actual, expected in zip(map(float, left.split()), map(float, right.split())):
            self.assertAlmostEqual(actual, expected, places=places)

    def test_total_mass(self):
        """总质量一致."""
        actual = sum(float(x.attrib['value']) for x in URDF.findall('link/inertial/mass'))
        expected = sum(float(x.attrib['mass']) for x in MJCF.findall('.//worldbody//inertial'))
        self.assertAlmostEqual(actual, expected, places=10)

    def test_link_inertias(self):
        """机身和每条腿各连杆的质量、质心位置和惯量张量一致."""
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
                full = expected.attrib.get('fullinertia', '').split()
                self.assertVectorEqual(
                    diag, expected.attrib.get('diaginertia', ' '.join(full[:3])))
                offdiag = ' '.join(values[key] for key in ('ixy', 'ixz', 'iyz'))
                self.assertVectorEqual(offdiag, ' '.join(full[3:]) if full else '0 0 0')

    def test_joint_frames_axes_and_limits(self):
        """各关节的安装位置、俯仰角、转轴方向和限位一致."""
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
                    self.assertVectorEqual(
                        joint.find('axis').attrib['xyz'], default.attrib['axis'])
                    limits = body.find('joint').attrib.get('range', default.attrib.get('range'))
                    self.assertVectorEqual(joint.find('limit').attrib['lower'] + ' ' +
                                           joint.find('limit').attrib['upper'], limits)


if __name__ == '__main__':
    unittest.main()
