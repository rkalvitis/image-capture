"""Runs inside the interpreter that has pymobiledevice3 (pipx venv or any env).
    python pull_worker.py list <bundle> <session>
    python pull_worker.py pull <bundle> <session> <images_dir> <sidecar_dir> <name> [<name> ...]
    python pull_worker.py delete <bundle> <session> <name> [<name> ...]     (after a verified cloud upload only)
Prints one JSON line with the result. Supports both the async API
(pymobiledevice3 >= 5: `await create_using_usbmux()`, `HouseArrestService.create`)
and the older sync one."""
import asyncio
import inspect
import json
import os
import sys


async def _open(bundle):
    from pymobiledevice3.lockdown import create_using_usbmux
    from pymobiledevice3.services.house_arrest import HouseArrestService
    ld = create_using_usbmux()
    if inspect.isawaitable(ld):
        ld = await ld
    if hasattr(HouseArrestService, "create"):
        afc = HouseArrestService.create(ld, bundle, documents_only=True)
        if inspect.isawaitable(afc):
            afc = await afc
    else:
        afc = HouseArrestService(lockdown=ld, bundle_id=bundle, documents_only=True)
    return afc


async def _call(f, *a, **k):
    r = f(*a, **k)
    return await r if inspect.isawaitable(r) else r


async def _resolve_base(afc, session):
    for base in (f"/{session}", f"/Documents/{session}"):
        try:
            await _call(afc.listdir, base)
            return base
        except Exception:
            continue
    raise FileNotFoundError(f"session folder {session!r} not found in the app's Documents")


async def main_async():
    cmd, bundle, session = sys.argv[1:4]
    afc = await _open(bundle)
    try:
        base = await _resolve_base(afc, session)
        if cmd == "list":
            names = await _call(afc.listdir, base)
            print(json.dumps({"files": sorted(names)}))
            return
        if cmd == "delete":
            deleted, missing = [], []
            rm = getattr(afc, "rm_single", None) or afc.rm
            for name in sys.argv[4:]:
                path = f"{base}/{name}"
                if not await _call(afc.exists, path):
                    missing.append(name)
                    continue
                await _call(rm, path)
                if await _call(afc.exists, path):
                    raise RuntimeError(f"still on the phone after rm: {name}")
                deleted.append(name)
            print(json.dumps({"deleted": deleted, "missing": missing}))
            return
        images_dir, sidecar_dir = sys.argv[4:6]
        out = []
        for name in sys.argv[6:]:
            dst_dir = sidecar_dir if name.endswith(".json") else images_dir
            dst = os.path.join(dst_dir, name)
            data = await _call(afc.get_file_contents, f"{base}/{name}")
            with open(dst + ".part", "wb") as f:
                f.write(data)
            os.replace(dst + ".part", dst)
            out.append(dst)
        print(json.dumps({"pulled": out}))
    finally:
        try:
            await _call(afc.close)
        except Exception:
            pass


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
