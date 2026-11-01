# NARA-IHA ArduPilot (TEKNOFEST Savaşan İHA)

`savasan` dalı: ArduPlane **Plane-4.7.2** üzerine takımın eklemeleri. Uçuş yazılımı (Jetson) NARA-IHA/savasan-ucak.

## Eklemeler

Hepsi `ArduPlane/savasan.{h,cpp}` (`Savasan` sınıfı, `SVS_` parametreleri, `SVS` log mesajı); mevcut dosyalarda yalnız
bağlantı satırları (`Plane.h`, `Plane.cpp` zamanlayıcı 10 Hz, `Parameters.*`, `servos.cpp` flap + sonlandırmada crow,
`mode.cpp`, `GCS_MAVLink_Plane.cpp`).

**Stok firmware'de aynısı:** savasan-ucak `ardupilot/scripts/savasan.lua` aynı parametre adları, komutlar ve mesajlarla
aynı işleri yapar (iki firmware de `sim/hangar/svs_testi.py` ve `failsafe_testi.py`'den geçer). Bu firmware'de
`SVS_FORK` olduğu için betik kendini kapatır. Ortak ArduPlane ayarları (şartname failsafe'i vb.):
savasan-ucak `ardupilot/savasan_ortak.param`.

| Ne | Nasıl | Parametreler |
|---|---|---|
| **GUIDED harici flap isteği** | `COMMAND_LONG MAV_CMD_USER_1` (31010), param1 = flap girişi 0..1 (RC flap kanalı ölçeği; `DSPOILER_OPTS 4` ile üst yarı crow). Yalnız GUIDED'da (değilse `DENIED`); `GUIDED_TIMEOUT` sonunda düşer, mod değişince ve Jetson kopunca silinir. Otomatik flap ve pilotun flap kanalıyla birleşir (büyük olan). Stok firmware'deki karşılığı savasan-ucak `ardupilot/scripts/fren.lua`. | `GUIDED_TIMEOUT` |
| **Dalışta hız freni** | Hava hızı (EAS) `SVS_OSPD`'yi aşınca flap/crow, `SVS_OSPD + SVS_OSPD_BAND`'da `SVS_OSPD_FLAP`. MANUAL dışında her modda, Jetson'dan bağımsız. Flapların yapısal hız sınırına dikkat. | `SVS_OSPD` (m/s, 0 = kapalı), `SVS_OSPD_BAND` (4), `SVS_OSPD_FLAP` (%100) |
| **Jetson kopma bekçisi** | Jetson'ın HEARTBEAT'i (aynı sysid, `SVS_CC_COMPID`) `SVS_CC_TOUT` s gelmezse `SVS: JETSON KOPTU` (kritik), flap isteği silinir; GUIDED'da ve arm'lıyken `SVS_CC_RTL` s daha sürerse RTL (her kopmada bir kez; geri gelince "Jetson geri geldi", mod pilotta kalır). Hiç görülmemişse devre dışı. GUIDED kendi başına `GUIDED_TIMEOUT`'ta guided noktasında loiter'a döner. | `SVS_CC_COMPID` (191), `SVS_CC_TOUT` (3 s), `SVS_CC_RTL` (0 = kapalı; RTL home'a düz uçar, HSS'den geçebilir) |
| **Kalkış öncesi Jetson** | Jetson görünmüyorsa "SVS: Jetson yok", heartbeat `system_status` ACTIVE değilse (sınır yüklü değil / kamera akmıyor) "SVS: Jetson hazir degil" ile arm reddedilir (aux auth; `ARMING_SKIPCHK` AuxAuth bitini atlıyorsa denetlenmez). | `SVS_ARM_JTSN` (1) |
| **Sonlandırmada crow yok** | AFS sonlandırması tam flap verir; kademeli crow'da bu iki kanatçığı da kaldırıp şartnamenin "tam sağ kanatçık"ını yarıya indirir → sonlandırma sürerken flap girişi %50'de kesilir (flaplar aşağı, crow yok). `servos.cpp dspoiler_update`. | — |
| **`SVS_FORK`** | Salt okunur işaret (1): stok firmware betiği `savasan.lua` bu firmware'de kendini kapatır. | — |
| **Yer testi** | `MAV_CMD_USER_2` (31011) param1 = 1 başlat / 0 durdur; yalnız disarm'da. 4 s'lik adımlar: nötr → FLAP (iç flaplar aşağı) → CROW (dış kanatçıklar da yukarı) → nötr; her adım STATUSTEXT. Arm edilirse durur. | — |
| **`SVS` log** | 10 Hz: `Req` (geçerli Jetson isteği %), `Os` (hız freni %), `Tst` (yer testi adımı), `CC` (0 görülmedi / 1 tamam / 2 koptu). | — |

Hangar testi: savasan-ucak `.venv/bin/python -m sim.hangar.svs_testi fork` (yer testi, guided flap, kalkış + 40° dalış 33 m/s'de crow,
Jetson kopması → 3 s'de uyarı, 8 s'de RTL, geri geliş).

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
