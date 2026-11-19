"""ONVIF PTZ zoom control for camera at 192.168.1.123.

Usage:
  python zoom.py in  [秒] [速度]   # ズームイン (望遠)  例: python zoom.py in 1.0 0.5
  python zoom.py out [秒] [速度]   # ズームアウト (広角)
  python zoom.py stop

Note: このカメラは AbsoluteMove 非対応 (送ると接続を切られる)。
      GetStatus のズーム位置も常に 0 を返すので位置フィードバックは無い。
"""
import sys
import time
import urllib.request

CAM = "http://192.168.1.123/onvif/ptz"
PROFILE = "MainStream"

def soap(body: str) -> str:
    env = ('<?xml version="1.0"?><s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
           'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" xmlns:tt="http://www.onvif.org/ver10/schema">'
           f'<s:Body>{body}</s:Body></s:Envelope>')
    req = urllib.request.Request(CAM, env.encode(), {"Content-Type": "application/soap+xml"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.read().decode(errors="ignore")

def continuous(speed: float):
    return soap(f'<tptz:ContinuousMove><tptz:ProfileToken>{PROFILE}</tptz:ProfileToken>'
                f'<tptz:Velocity><tt:Zoom x="{speed}"/></tptz:Velocity></tptz:ContinuousMove>')

def stop():
    return soap(f'<tptz:Stop><tptz:ProfileToken>{PROFILE}</tptz:ProfileToken>'
                '<tptz:PanTilt>true</tptz:PanTilt><tptz:Zoom>true</tptz:Zoom></tptz:Stop>')

def absolute(pos: float):
    return soap(f'<tptz:AbsoluteMove><tptz:ProfileToken>{PROFILE}</tptz:ProfileToken>'
                f'<tptz:Position><tt:Zoom x="{pos}"/></tptz:Position></tptz:AbsoluteMove>')

def status():
    import re
    m = re.search(r'<tt:Zoom x="([^"]*)"', soap(
        f'<tptz:GetStatus><tptz:ProfileToken>{PROFILE}</tptz:ProfileToken></tptz:GetStatus>'))
    return m.group(1) if m else None

def main():
    a = sys.argv[1:] or ["status"]
    cmd = a[0]
    if cmd in ("in", "out"):
        dur = float(a[1]) if len(a) > 1 else 1.0
        spd = float(a[2]) if len(a) > 2 else 0.5
        continuous(spd if cmd == "in" else -spd)
        time.sleep(dur)
        stop()
    elif cmd == "stop":
        stop()
    else:
        print(__doc__)
        return
    print("ok")

if __name__ == "__main__":
    main()
