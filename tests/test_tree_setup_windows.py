#!/usr/bin/env python
"""Check Windows setup timeout handling in an isolated interpreter."""

import subprocess
import sys
import textwrap

import pytest


@pytest.mark.skipif(sys.platform != "win32", reason="Windows signal delivery regression")
def test_setup_timeout_does_not_terminate_process() -> None:
    """A setup timeout must raise RuntimeError and restore the signal handler."""
    program = textwrap.dedent(
        """
        import signal
        import time
        import py_trees

        class SlowSetup(py_trees.behaviours.Success):
            def setup(self, **kwargs):
                time.sleep(0.1)

        root = SlowSetup(name="slow")
        original_handler = signal.getsignal(signal.SIGINT)
        try:
            py_trees.trees.setup(root=root, timeout=0.02)
        except RuntimeError as error:
            assert "timed out [slow]" in str(error), str(error)
        else:
            raise AssertionError("expected a setup timeout")
        assert signal.getsignal(signal.SIGINT) == original_handler
        py_trees.trees.setup(root=root)
        print("timeout handled; subsequent setup completed")
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", program],
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    assert "timeout handled; subsequent setup completed" in result.stdout
