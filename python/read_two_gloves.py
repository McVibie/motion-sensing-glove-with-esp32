import asyncio
import csv
import time
from bleak import BleakScanner, BleakClient

TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

SCAN_TIMEOUT = 12.0

NAME_PREFIX = "ESP32_GLOVE"

AUTO_SEND_CAL = True

LOG_TO_CSV = True
CSV_FILE = "two_gloves_log.csv"

_csv_f = None
_csv_w = None


def ensure_csv():
    global _csv_f, _csv_w
    if not LOG_TO_CSV or _csv_w is not None:
        return

    _csv_f = open(CSV_FILE, "a", newline="")
    _csv_w = csv.writer(_csv_f)
    if _csv_f.tell() == 0:
        _csv_w.writerow(["ts", "glove", "line"])
        _csv_f.flush()


def csv_write(glove_tag: str, line: str):
    ensure_csv()
    if _csv_w is None:
        return
    _csv_w.writerow([time.time(), glove_tag, line])
    _csv_f.flush()


def make_notify_handler(glove_tag: str):
    def _handle(_, data: bytearray):
        line = data.decode(errors="ignore").strip()
        if not line:
            return
        print(f"[{glove_tag}] {line}")
        if LOG_TO_CSV:
            csv_write(glove_tag, line)
    return _handle


async def pick_two_devices():
    print(f"Scanning {SCAN_TIMEOUT}s for devices starting with '{NAME_PREFIX}' ...")
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)

    matches = []
    for d in devices:
        name = d.name or ""
        if name.startswith(NAME_PREFIX):
            matches.append(d)

    # remove duplicates by address
    uniq = {}
    for d in matches:
        uniq[d.address] = d
    matches = list(uniq.values())

    if len(matches) < 2:
        print("Found devices:")
        for d in matches:
            print(f"  - {d.name} ({d.address})")
        return None, None

    # pick first two
    a, b = matches[0], matches[1]
    print("Selected:")
    print(f"  [L] {a.name} ({a.address})")
    print(f"  [R] {b.name} ({b.address})")
    return a, b


async def run_one(glove_tag: str, address: str):
    while True:
        try:
            async with BleakClient(address) as client:
                print(f"[{glove_tag}] Connected: {address}")
                await client.start_notify(TX_CHAR_UUID, make_notify_handler(glove_tag))
                print(f"[{glove_tag}] Subscribed to TX")

                if AUTO_SEND_CAL:
                    await client.write_gatt_char(RX_CHAR_UUID, b"CAL\n", response=False)
                    print(f"[{glove_tag}] Sent: CAL")

                while True:
                    await asyncio.sleep(1)

        except Exception as e:
            print(f"[{glove_tag}] Error/disconnect: {e} -> reconnecting...")
            await asyncio.sleep(1)


async def main():
    left, right = await pick_two_devices()
    if not left or not right:
        print("Need 2 gloves visible in scan. Turn both on and retry.")
        return

    tasks = [
        asyncio.create_task(run_one("L", left.address)),
        asyncio.create_task(run_one("R", right.address)),
    ]
    try:
        await asyncio.gather(*tasks)
    finally:
        global _csv_f
        if _csv_f:
            _csv_f.close()


if __name__ == "__main__":
    asyncio.run(main()) 