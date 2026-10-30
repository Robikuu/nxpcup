# Check how vectors.c behaves with the raw telemetry values
# In the transcript, we also have raw dump vectors if g_dump_vectors was on, but was it on?
# Let's search for RAW_TOP in the transcript
with open('d:/nxpcup/latest_user_request.txt', 'r', encoding='utf-8') as f:
    text = f.read()

print('RAW_TOP in transcript?', 'RAW_TOP' in text)
