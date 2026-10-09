from __future__ import annotations
import json
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from multi_motor.export_feedback import export_feedback


class FeedbackExportTests(unittest.TestCase):
    def test_per_motor_timestamped_snapshot_preserves_raw_measured_and_reference_values(self):
        samples = [(100.0, 50.4, 9.63, 0.35, 10.0, 10.37, 51.0, 1, '11 80 00 80 08 00 19 1E', 1791435600.0, 0.4)]
        saved = datetime(2026, 10, 8, 14, 5, 7, tzinfo=timezone(timedelta(hours=7)))
        payloads = []

        def fake_process(args, **kwargs):
            payload = json.loads(Path(args[2]).read_text(encoding='utf-8'))
            payloads.append(payload)
            for suffix in payload['formats']:
                Path(payload['base'] + '.' + suffix).write_bytes(b'exported')
            return SimpleNamespace(returncode=0, stderr='')

        with tempfile.TemporaryDirectory() as folder:
            with patch('multi_motor.export_feedback.export_runtime', return_value=(Path('node.exe'), Path('modules'))), patch('multi_motor.export_feedback.subprocess.run', side_effect=fake_process):
                one = export_feedback(1, samples, 50.4, output_dir=folder, saved_at=saved)
                two = export_feedback(2, samples, 50.4, output_dir=folder, saved_at=saved)
                repeat = export_feedback(1, samples, 50.4, output_dir=folder, saved_at=saved)
            self.assertEqual(len(one), 2)
            self.assertIn('motor_0x01_2026-10-08_14-05-07', one[0].name)
            self.assertIn('motor_0x02', two[0].name)
            self.assertNotEqual(one, repeat)
            row = payloads[0]['rows'][0]
            self.assertEqual(row[2:7], [0.0, 9.63, 10.0, 10.37, 0.35])
            self.assertAlmostEqual(row[-1], 0.6)
            self.assertEqual(samples[0][2], 9.63)

    def test_empty_feedback_does_not_create_a_false_export(self):
        with self.assertRaises(ValueError):
            export_feedback(1, [], 0)


if __name__ == '__main__':
    unittest.main()
