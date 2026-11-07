"""Launch either curated version; --offline never opens physical devices."""
import argparse
import importlib
import sys
from software import settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", choices=("v1", "v2"), default="v2")
    parser.add_argument("--config", help="JSON settings file")
    parser.add_argument("--offline", action="store_true", help="No cameras, serial or robot connections")
    parser.add_argument("--smoke-test", action="store_true", help="Offline UI startup and automatic shutdown")
    args = parser.parse_args()
    settings.configure(args.config, offline=args.offline or args.smoke_test)
    try:
        module = importlib.import_module(f"software.{args.version}")
    except ImportError as exc:
        parser.exit(2, f"Missing dependency: {exc}\nInstall requirements-{args.version}.txt first.\n")
    app = module.QApplication([sys.argv[0]])
    from software.desktop import setup_fonts
    setup_fonts(app, module.QFontDatabase, module.QFont)
    window = module.MainWindow()
    window.show()
    if args.smoke_test:
        module.QTimer.singleShot(800, window.close)
        module.QTimer.singleShot(1200, app.quit)
    return app.exec() if args.version == "v2" else app.exec_()


if __name__ == "__main__":
    sys.exit(main())
