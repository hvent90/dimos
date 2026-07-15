# Copyright 2026 Dimensional Inc.

from __future__ import annotations

import argparse
import base64
import os
import pickle
import signal
import threading
import time

from dimos.core.external_python_module import ExternalPythonModule
from dimos.core.module import Module


def _load(ref: str) -> type:
    module_name, separator, class_name = ref.partition(":")
    if not separator:
        module_name, _, class_name = ref.rpartition(".")
    if not module_name or not class_name:
        raise ValueError(f"Invalid import reference {ref!r}; use module:Class")
    try:
        module = __import__(module_name, fromlist=[class_name])
        value = getattr(module, class_name)
    except (ImportError, AttributeError) as error:
        raise ImportError(
            f"Could not load class {class_name!r} from {module_name!r}: {error}"
        ) from error
    if not isinstance(value, type):
        raise TypeError(f"Import reference {ref!r} does not resolve to a class")
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--declaration", required=True)
    parser.add_argument("--implementation", required=True)
    parser.add_argument("--handshake-fd", required=True, type=int)
    parser.add_argument("--kwargs", required=True)
    args = parser.parse_args()
    handshake_open = True
    try:
        declaration = _load(args.declaration)
        implementation = _load(args.implementation)
        if not issubclass(declaration, ExternalPythonModule):
            raise TypeError(f"Declaration {args.declaration!r} is not an ExternalPythonModule")
        if not issubclass(implementation, Module):
            raise TypeError(f"Implementation {args.implementation!r} is not a Module subclass")
        if not issubclass(implementation, declaration):
            raise TypeError(
                f"Implementation {args.implementation!r} does not implement {args.declaration!r}"
            )
        kwargs = pickle.loads(base64.b64decode(args.kwargs))
        module = implementation(**kwargs)
        if module.rpc is None:
            raise RuntimeError(f"Implementation {args.implementation!r} has no RPC server")
        module.rpc.serve_module_rpc(module, name=declaration.__name__)
        os.write(args.handshake_fd, b"READY\n")
    except Exception as error:
        handshake_open = False
        try:
            os.write(
                args.handshake_fd,
                f"ERROR {type(error).__name__}: {error}\n".encode(),
            )
        except OSError:
            handshake_open = False
        raise
    finally:
        if handshake_open:
            try:
                os.close(args.handshake_fd)
            except OSError:
                pass

    stopping = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stopping.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    while not stopping.is_set():
        time.sleep(1)


if __name__ == "__main__":
    main()
