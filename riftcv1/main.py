"""Entry point: no arguments → GTK GUI, any arguments → CLI."""
import sys


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv:
        from .cli import main as cli_main
        return cli_main(argv)
    from .gui import run_gui
    return run_gui()
