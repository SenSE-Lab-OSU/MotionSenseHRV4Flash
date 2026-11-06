"""Exercise production brightness bookkeeping with host acquisition inputs."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


class PpgBrightnessCounterTest(unittest.TestCase):
    def test_counter_empty_reads_and_collection_restart(self):
        src = Path(__file__).resolve().parents[2] / "src"
        source = (src / "ppgSensor.c").read_text()
        header = (src / "ppgSensor.h").read_text()

        def extract(pattern, text=source):
            match = re.search(pattern, text)
            self.assertIsNotNone(match, pattern)
            return match.group(0)

        declarations = "\n".join([
            extract(r"#define PPG_FIXED_TIME_WINDOW[^\n]+", header),
            extract(r"uint\d+_t timeWindow\s*=\s*[^;]+;"),
            extract(r"uint\d+_t ppg_brightness_check_counter\s*=\s*[^;]+;"),
        ] + re.findall(r"float32_t runningMeanCh[^\n]+", source))
        advance = extract(
            r"ppg_brightness_check_counter\+\+;\s*"
            r"if\s*\([^)]*\)\s*\{[^}]*\}"
        )
        gate = extract(r"if\s*\(ppg_brightness_check_counter == timeWindow\)")
        empty = extract(r"if\s*\(number_of_samples == 0\)\s*\{[^}]*\}")
        # Preserve production ordering: moving the counter above the early return
        # must fail the empty-read cases, rather than being repaired by the test.
        acquisition = "\n".join(sorted([advance, empty], key=source.index))
        means = extract(r"(?s)if \(ppg_brightness_check_counter == 0\).*?"
                        r"runningMeanCh2b = runningMeanCh2b[^;]+;")
        config = source[source.index("void ppg_config()"):source.index("void ppg_changeIntensity")]
        reset = extract(r"(?s)ppg_brightness_check_counter = 0;.*?(?=badDataCounterCh1 = 0;)", config)
        harness = (Path(__file__).with_name("ppg_brightness_harness.c")).read_text()
        for marker, code in (("DECLARATIONS", declarations), ("ACQUISITION", acquisition),
                             ("MEANS", means), ("GATE", gate), ("RESET", reset)):
            harness = harness.replace("/* " + marker + " */", code)
        cc = r"C:\cygwin64\bin\gcc.exe" if os.name == "nt" else shutil.which("gcc")
        self.assertTrue(cc, "GCC is required")
        env = os.environ.copy()
        env["PATH"] = str(Path(cc).resolve().parent) + os.pathsep + env["PATH"]
        with tempfile.TemporaryDirectory(prefix="ppg-brightness-") as temp:
            output = Path(temp) / "harness.c"
            executable = Path(temp) / "harness.exe"
            output.write_text(harness)
            subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                            str(output), "-o", str(executable)], env=env, check=True)
            subprocess.run([str(executable)], env=env, check=True)


if __name__ == "__main__":
    unittest.main()
