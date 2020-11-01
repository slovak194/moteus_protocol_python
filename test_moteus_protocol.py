from scipy import nan as kNaN

import moteus_protocol as mp

import unittest


class TestMoteusProtocolPython(unittest.TestCase):

    def test_SaturateTest(self):
        self.assertEqual(mp.SaturateInt8(-1000.0, 1.0), -127)
        self.assertEqual(mp.SaturateInt8(1000.0, 1.0), 127)
        self.assertEqual(mp.SaturateInt8(kNaN, 1.0), -128)
        self.assertEqual(mp.SaturateInt8(10.0, 1.0), 10)
        self.assertEqual(mp.SaturateInt8(-15.0, 1.0), -15)
        self.assertEqual(mp.SaturateInt8(0.0, 1.0), 0)
        self.assertEqual(mp.SaturateInt8(10, 0.1), 100)

        self.assertEqual(mp.SaturateInt16(-1000000, 1.0), -32767)
        self.assertEqual(mp.SaturateInt16(kNaN, 1.0), -32768)
        self.assertEqual(mp.SaturateInt16(123, 1.0), 123)

    def test_EmitPositionCommandTest(self):
        f = mp.CanFrame()
        can_frame = mp.WriteCanFrame(f)

        pos = mp.PositionCommand()
        res = mp.PositionResolution()

        res.position = mp.Resolution.kInt8
        res.velocity = mp.Resolution.kIgnore
        res.feedforward_torque = mp.Resolution.kIgnore
        res.kp_scale = mp.Resolution.kIgnore
        res.kd_scale = mp.Resolution.kIgnore
        res.maximum_torque = mp.Resolution.kIgnore
        res.stop_position = mp.Resolution.kIgnore
        res.watchdog_timeout = mp.Resolution.kIgnore

        mp.EmitPositionCommand(can_frame, pos, res)

        self.assertTrue(f.size == 6)
        self.assertTrue(f.data[0] == 0x01)
        self.assertTrue(f.data[1] == 0x00)
        self.assertTrue(f.data[2] == 0x0a)
        self.assertTrue(f.data[3] == 0x01)
        self.assertTrue(f.data[4] == 0x20)
        self.assertTrue(f.data[5] == 0x00)

        # # Now try with more than one register of a different type after.
        res.velocity = mp.Resolution.kInt16
        res.feedforward_torque = mp.Resolution.kInt16
        res.kp_scale = mp.Resolution.kIgnore
        res.kd_scale = mp.Resolution.kFloat
        res.maximum_torque = mp.Resolution.kFloat
        res.stop_position = mp.Resolution.kFloat
        res.watchdog_timeout = mp.Resolution.kFloat

        pos.position = 0.4
        pos.velocity = 0.2
        pos.feedforward_torque = -1.0
        pos.kd_scale = 0.3
        pos.maximum_torque = 4.0
        pos.stop_position = 1.2
        pos.watchdog_timeout = 0.5

        f.size = 0
        f.data[:] = 0

        mp.EmitPositionCommand(can_frame, pos, res)
        self.assertTrue(f.size == 31)

        self.assertTrue(f.data[0] == 0x01)
        self.assertTrue(f.data[1] == 0x00)
        self.assertTrue(f.data[2] == 0x0a)
        self.assertTrue(f.data[3] == 0x01)
        self.assertTrue(f.data[4] == 0x20)
        self.assertTrue(f.data[5] == 0x28)

        self.assertTrue(f.data[6] == 0x06)  # write 2 int16
        self.assertTrue(f.data[7] == 0x21)  # starting at 21
        self.assertTrue(f.data[8] == 0x20)  # velocity 0.2 / 0.00025 = 0x320
        self.assertTrue(f.data[9] == 0x03)
        self.assertTrue(f.data[10] == 0x9c)  # torque -1.0 / 0.01 = 0xff9c
        self.assertTrue(f.data[11] == 0xff)

        self.assertTrue(f.data[12] == 0x0c)  # write float
        self.assertTrue(f.data[13] == 0x04)  # 4x floats
        self.assertTrue(f.data[14] == 0x24)  # starting at 24 (kd scale)

    def test_EmitQueryCommandTest(self):
        f = mp.CanFrame()
        can_frame = mp.WriteCanFrame(f)

        cmd = mp.QueryCommand()
        mp.EmitQueryCommand(can_frame, cmd)

        self.assertTrue(f.size == 5)
        self.assertTrue(f.data[0] == 0x14)
        self.assertTrue(f.data[1] == 0x04)
        self.assertTrue(f.data[2] == 0x00)
        self.assertTrue(f.data[3] == 0x13)
        self.assertTrue(f.data[4] == 0x0d)

    def test_ParseQueryResultTest(self):
        can_frame = mp.CanFrame()
        can_frame.size = 5
        can_frame.data[0] = 0x23
        can_frame.data[1] = 0x00
        can_frame.data[2] = 0x01
        can_frame.data[3] = 0x02
        can_frame.data[4] = 0x03

        result = mp.ParseQueryResult(can_frame)
        self.assertTrue(result.mode == mp.Mode.kFault)
        self.assertTrue(result.position == 0.02)
        self.assertTrue(result.velocity == 0.30000000000000004)

        can_frame.size = 13

        # Verify we can skip a nop in the middle.
        can_frame.data[5] = 0x50

        can_frame.data[6] = 0x24  # n int16s
        can_frame.data[7] = 0x02  # 2 of them
        can_frame.data[8] = 0x0d  # starting at voltage
        can_frame.data[9] = 0x40  # 6.4V
        can_frame.data[10] = 0x00
        can_frame.data[11] = 0x50  # 8.0C
        can_frame.data[12] = 0x00

        result = mp.ParseQueryResult(can_frame)
        self.assertTrue(result.mode == mp.Mode.kFault)
        self.assertTrue(result.position == 0.02)
        self.assertTrue(result.velocity == 0.30000000000000004)
        self.assertTrue(result.voltage == 6.4)
        self.assertTrue(result.temperature == 8.0)


if __name__ == '__main__':
    unittest.main()
