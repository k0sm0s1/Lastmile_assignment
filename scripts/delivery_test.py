#!/usr/bin/env python3
"""
End-to-end delivery check through the delivery page's own HTTP API
(run while `DELIVERY=1 bash scripts/run_demo.sh` is up; add OBSTACLES=1 for the slalom).

Starts a mission, "presses" Loaded / Unloaded when the page enables them, and
prints the timeline. Usage: python3 scripts/delivery_test.py [pickup_id] [dropoff_id]
"""
import json
import sys
import time
import urllib.request

BASE = "http://localhost:8081"


def post(path, body=None):
    req = urllib.request.Request(BASE + path, data=json.dumps(body or {}).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        return json.loads(urllib.request.urlopen(req, timeout=10).read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


def snapshot():
    with urllib.request.urlopen(BASE + "/api/stream", timeout=10) as r:
        for line in r:
            if line.startswith(b"data: "):
                return json.loads(line[6:])


def main():
    pk_id, dp_id = (sys.argv[1], sys.argv[2]) if len(sys.argv) > 2 else ("mailroom", "office")
    places = {p["id"]: p for p in json.loads(urllib.request.urlopen(BASE + "/api/map").read())["places"]}
    pk, dp = places[pk_id], places[dp_id]
    print("start:", post("/api/mission", {"pickup": pk, "dropoff": dp}), flush=True)
    t0, last = time.time(), None
    while time.time() - t0 < 1500:
        s = snapshot()
        ms = s.get("mission") or {}
        st = ms.get("state")
        if st != last:
            print(f"{time.time() - t0:6.0f}s  {st}  robot={s['robot']['xyz'] and [round(v, 2) for v in s['robot']['xyz'][:2]]}", flush=True)
            last = st
        if st == "at_pickup":
            time.sleep(2); print("        press Loaded:", post("/api/loaded"), flush=True)
        elif st == "at_dropoff":
            time.sleep(2); print("        press Unloaded:", post("/api/unloaded"), flush=True)
        elif st in ("delivered", "problem"):
            print(json.dumps({"state": st, "duration_s": ms.get("duration_s"), "problem": ms.get("problem"),
                              "timeline": [(e["state"], round(e["t"] - ms["timeline"][0]["t"])) for e in ms["timeline"]]}))
            return
        time.sleep(1.5)


if __name__ == "__main__":
    main()
