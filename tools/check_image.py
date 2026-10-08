#!/usr/bin/env python3
"""Audit a prepared image against its exact baseline and built binary, offline."""
import argparse
import json
from pathlib import Path
import sys

from package_app import (
    APP_LIMIT, CONTAINER_PREFIX, ImageError, audit, check_application,
    confirmed_bytes, verify_container,
)


def check(image):
    report = verify_container(image)
    app = image[len(CONTAINER_PREFIX):len(CONTAINER_PREFIX) + APP_LIMIT]
    report.update(check_application(app))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--confirm-sha256", required=True)
    parser.add_argument("--app", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--baseline-sha256", required=True)
    args = parser.parse_args()
    try:
        image = confirmed_bytes(args.image, args.confirm_sha256)
        baseline = confirmed_bytes(args.baseline, args.baseline_sha256)
        report = check(image)
        report.update(audit(image, baseline, args.app.read_bytes()))
        report["status"] = "structural audit passed; hardware not validated"
        print(json.dumps(report, indent=2))
        return 0
    except (OSError, ImageError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
