"""
******************************************************************************
 * @file : test_phone_link.py
 * @brief : Phone discovery and command-link target selection.
 * @author : David Nguyen
******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 MRacing. All rights reserved.
 * MRacing is a trademark of MRacing FSAE.
 *
 * Written by David Nguyen.
 *
 * This firmware is the property of MRacing FSAE. Unauthorized use, copying,
 * or distribution is prohibited.
 *
******************************************************************************
"""
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gsdash.server import PhoneLink, Telemetry, parse_browse  # noqa: E402

BROWSE = """Browsing for _groundspeed._tcp.local.
DATE: ---Sun 04 Oct 2026---
 1:10:51.507  ...STARTING...
Timestamp     A/R    Flags  if Domain               Service Type         Instance Name
 1:10:51.507  Add        2  11 local.               _groundspeed._tcp.   iPhone
23:36:51.002  Add        2  11 local.               _groundspeed._tcp.   iPhone GroundSpeed
"""


class ParseBrowseTest(unittest.TestCase):
    def test_single_and_double_digit_hours(self):
        self.assertEqual(parse_browse(BROWSE), ["iPhone", "iPhone GroundSpeed"])


class TargetTest(unittest.TestCase):
    def setUp(self):
        self.telem = Telemetry()
        self.link = PhoneLink(self.telem, bonjour=False)

    def tearDown(self):
        self.link.close()

    def test_stale_udp_source_falls_back_to_bonjour(self):
        self.telem.source_ip = "2607::1"
        self.telem.last_recv = time.time() - 10
        self.link.bonjour_result = ("2607::2", 9001)
        self.assertEqual(self.link.target(), ("2607::2", 9001, "bonjour"))

    def test_fresh_udp_source_wins(self):
        self.telem.source_ip = "2607::1"
        self.telem.last_recv = time.time()
        self.link.bonjour_result = ("2607::2", 9001)
        self.assertEqual(self.link.target()[2], "udp-source")


if __name__ == "__main__":
    unittest.main()
