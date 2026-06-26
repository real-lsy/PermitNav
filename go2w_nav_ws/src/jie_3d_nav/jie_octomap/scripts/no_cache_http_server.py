#!/usr/bin/env python3

from __future__ import annotations

import argparse
import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


class NoCacheHTTPRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve static files without browser caching.")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--directory", required=True)
    parser.add_argument("--bind", default="0.0.0.0")
    args = parser.parse_args()

    handler = functools.partial(NoCacheHTTPRequestHandler, directory=args.directory)
    server = ThreadingHTTPServer((args.bind, args.port), handler)
    print(
        f"Serving HTTP no-cache on {args.bind} port {args.port} "
        f"(directory: {args.directory})",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
