# NARA-IHA ArduPilot (TEKNOFEST Savaşan İHA)

`savasan` dalı: ArduPlane **Plane-4.7.2** üzerine takımın eklemeleri. Uçuş yazılımı (Jetson) NARA-IHA/savasan-ucak.

## Eklemeler

| Ne | Nerede | Not |
|---|---|---|
| GUIDED harici flap isteği | `ArduPlane/GCS_MAVLink_Plane.cpp`, `servos.cpp` | `COMMAND_LONG MAV_CMD_USER_1` (31010), param1 = flap girişi 0..1 (RC flap kanalı ölçeği). Yalnız GUIDED'da kabul (değilse `DENIED`); `GUIDED_TIMEOUT` sonunda düşer, mod değişince silinir. Otomatik flap ve pilotun flap kanalıyla birleşir (büyük olan). Bertaraf'ta `DSPOILER_OPTS 4` ile üst yarı crow (hava freni). Stok firmware'de aynı iş `savasan-ucak/ardupilot/scripts/fren.lua` ile yapılır. |

## Derleme

```bash
uv venv ~/.venvs/ardupilot && uv pip install -p ~/.venvs/ardupilot empy==3.3.4 pymavlink future pexpect lxml setuptools
export PATH=~/.venvs/ardupilot/bin:$PATH
git submodule update --init --recursive
./waf configure --board sitl && ./waf plane          # SITL: build/sitl/bin/arduplane
./waf configure --board MatekH743 && ./waf plane     # uçak: build/MatekH743/bin/arduplane.apj
```

Hangar'da fork'u kullanmak: `HANGAR_CONFIG=~/.config/hangar/fork.toml` (`[ardupilot] bin_dir` → `build/sitl/bin`).

## Upstream güncelleme

```bash
git fetch upstream --tags && git rebase Plane-4.7.x savasan     # yeni kararlı sürüm
```
