with open(r"d:\nxpcup\scratch\latest_telemetry.txt", "rb") as f:
    data = f.read(500)
    print("Raw bytes:", repr(data))
