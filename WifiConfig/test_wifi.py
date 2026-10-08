# check of the networks.txt handling in wifi.py, runs anywhere:  python test_wifi.py
import wifi

nets = wifi.parse("# comment\n\non, Home, pass,with,commas\noff, iPhone, hunter22\non, eduroam\n")
assert [n["ssid"] for n in nets] == ["Home", "iPhone", "eduroam"]
assert nets[0]["password"] == "pass,with,commas" and nets[2]["password"] == ""
assert [n["on"] for n in nets] == [True, False, True]
assert wifi.parse(wifi.dump(nets)) == nets

assert wifi.find(nets, "iph")["ssid"] == "iPhone"
assert wifi.find(nets, "HOME")["ssid"] == "Home"
wifi.move(nets, wifi.find(nets, "edu"), 1)
assert [n["ssid"] for n in nets] == ["eduroam", "Home", "iPhone"]
wifi.move(nets, nets[0], 99)
assert nets[-1]["ssid"] == "eduroam"

for bad in ("maybe, Home, x", "Home", "on, , x"):
    try:
        wifi.parse(bad)
        raise AssertionError(bad)
    except SystemExit:
        pass
try:
    wifi.find(nets, "o")    # Home, iPhone and eduroam all match
    raise AssertionError
except SystemExit:
    pass
print("ok")
