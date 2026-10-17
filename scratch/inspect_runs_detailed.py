with open('scratch/full_user_msg.txt') as f:
    text = f.read()

parts = text.split('OK state=RUNNING')

for i, part in enumerate(parts[1:], 1):
    lines = [l.strip() for l in part.split('\n') if l.strip().startswith('L:')]
    print(f"\n==================== RUN {i} ({len(lines)} frames) ====================")
    # Print in intervals or key transitions
    for idx, line in enumerate(lines):
        # Print when speed changes or heading changes significantly or near the end
        if idx < 10 or idx > len(lines) - 20 or idx % 10 == 0:
            print(f"{idx:3d}: {line}")
