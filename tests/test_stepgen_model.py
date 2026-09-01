import unittest


UINT32 = 1 << 32
ISR_HZ = 30000
TICKS_PER_COMMAND = 30
MAX_INCREMENT = UINT32 // 2


class AxisModel:
    """Small reference model for v2 phase retention and pulse spacing."""

    def __init__(self):
        self.position = 0
        self.target = 0
        self.acc = 0
        self.inc = 0
        self.direction = 1
        self.low = 0
        self.setup = 0
        self.hold = 0
        self.pulses = []
        self.tick_number = 0

    def retarget(self, target):
        direction = -1 if target < self.position else 1
        if target != self.position and direction != self.direction:
            self.acc = 0
            self.direction = direction
            self.setup = 1
        self.target = target
        distance = min(abs(target - self.position), 15)
        self.inc = distance * UINT32 // TICKS_PER_COMMAND

    def tick(self):
        self.tick_number += 1
        blocked = self.low or self.setup or self.hold
        self.low = max(0, self.low - 1)
        self.setup = max(0, self.setup - 1)
        self.hold = max(0, self.hold - 1)
        if self.position == self.target:
            return
        old = self.acc
        self.acc = (self.acc + self.inc) % UINT32
        if self.acc < old and not blocked:
            self.position += self.direction
            self.pulses.append(self.tick_number)
            self.low = self.hold = 1


class StepgenModelTests(unittest.TestCase):
    def test_retargets_do_not_starve_slow_step(self):
        axis = AxisModel()
        for tick in range(120):
            if tick % TICKS_PER_COMMAND == 0:
                axis.retarget(axis.position + 1)
            axis.tick()
        self.assertGreaterEqual(axis.position, 3)

    def test_max_rate_has_complete_low_tick(self):
        axis = AxisModel()
        axis.target = 100
        axis.inc = MAX_INCREMENT
        for _ in range(30):
            axis.tick()
        gaps = [b - a for a, b in zip(axis.pulses, axis.pulses[1:])]
        self.assertTrue(gaps)
        self.assertGreaterEqual(min(gaps), 2)

    def test_reversal_has_setup_delay(self):
        axis = AxisModel()
        axis.position = 10
        axis.retarget(0)
        axis.inc = MAX_INCREMENT
        axis.tick()
        self.assertEqual(axis.pulses, [])


if __name__ == "__main__":
    unittest.main()
