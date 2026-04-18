# protocol.py — stateless helpers, easy to unit-test
def calc_crc(body: str) -> str:
    return f"{sum(ord(c) for c in body) & 0xFF:02X}"


def build_cmd(body: str) -> bytes:
    return (body + calc_crc(body)).encode("ascii")


def decode_frame(resp: str) -> dict | None:
    if len(resp) < 12:
        return None

    try:
        length = int(resp[5], 16)
    except ValueError:
        return None

    body = resp[:-2]
    return {
        "header": resp[0:3],
        "src": resp[3],
        "dst": resp[4],
        "length": length,
        "control": resp[6],
        "cmd": resp[7:10],
        "data": resp[10:-2],
        "crc_ok": resp[-2:].upper() == calc_crc(body).upper(),
    }
