import pickle

def message_to_string(msg) -> bytes:
    return pickle.dumps(msg)

def string_to_message(string: bytes) -> object:
    return pickle.loads(string)