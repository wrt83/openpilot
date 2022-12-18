#!/usr/bin/env python3
# You might need to uninstall the PyQt5 pip package to avoid conflicts

import os
import time
import numpy as np
from cffi import FFI

from common.ffi_wrapper import suffix
from common.basedir import BASEDIR

HEIGHT, WIDTH = 480, 768
METERS_PER_PIXEL = 2


def get_ffi():
  lib = os.path.join(BASEDIR, "selfdrive", "navd", "libmap_renderer" + suffix())

  ffi = FFI()
  ffi.cdef("""
void* map_renderer_init(char *maps_host, char *token);
void map_renderer_update_position(void *inst, float lat, float lon, float bearing);
void map_renderer_update_route(void *inst, char *polyline);
void map_renderer_update(void *inst);
void map_renderer_process(void *inst);
bool map_renderer_loaded(void *inst);
uint8_t* map_renderer_get_image(void *inst);
void map_renderer_free_image(void *inst, uint8_t *buf);
""")
  return ffi, ffi.dlopen(lib)


def wait_ready(lib, renderer):
  while not lib.map_renderer_loaded(renderer):
    lib.map_renderer_update(renderer)

    # The main qt app is not execed, so we need to periodically process events for e.g. network requests
    lib.map_renderer_process(renderer)

    time.sleep(0.01)


def get_image(lib, renderer):
  buf = lib.map_renderer_get_image(renderer)
  r = list(buf[0:WIDTH * HEIGHT*3])
  lib.map_renderer_free_image(renderer, buf)

  # Convert to numpy
  r = np.asarray(r)
  return r.reshape((HEIGHT, WIDTH, 3))


if __name__ == "__main__":
  import matplotlib.pyplot as plt

  ffi, lib = get_ffi()
  renderer = lib.map_renderer_init(ffi.NULL, ffi.NULL)
  wait_ready(lib, renderer)

  geometry = r"ulzk}@fm{m~Eu@?oH?cB?{@?sAN}@Oya@NaH?_D?gD?uEL_D?kV?wRNmO?y\?mOLmA?_I?m@?uD?yC?iR?cVNaM?gC?mFNwR?qG?W?{ULwC?gE?_N^iG???oJgd@wMmi@sPap@aHwY}To|@eAuD{@uE]eEyBuZ}@kKqLku@u@}Te@kLmEwv@{Am_@?yVbA{VlAkLdDoRfEoSvH}T~I_T??jPyMxR{UzK}JrPiLxRkLzU}Jj`@gNjQcFt^oIf]cGfs@iMdTuDp\wCpR}@lZ?nfAdFnYjAvM\dJ\^?`M?tTuDtOwDhGyB`HiCxHsE`HsF`HqH`H_IxG_J??dUgm@rFyWrFy`@pBq\|DgkBMgm@kAy{EOuvBGsmAUq{A?c[?_]Wud@{Auc@eDqg@aCse@_Dse@yCc[kAkL{AyLyGoh@iLms@e@iC_O_r@sLcf@wHq\gDcPe_@ofB??iGqf@iCa\kKaeByCee@?qQl@sPpBqRtPi`AhB}JbBqG`BsGjG{KpLoHrGiCxGOth@zA|TdFfT`H??`f@zUxg@bPrFjBvRlJfT|JnChBjBvCz@xB^hCDfDUvDcAvCmAxBe@\qBzAmEjA}I?yH]aG]wI_@??G`HGlJ?vN?xM?rE?tE]hXOjU?~IFzJ?lA?jAFfNTz_@rAdE?zAVhXFvCTdEjAhMl@pQFhCFjK?fDG`S?vNFfC?hC?jA?~I????"
  lib.map_renderer_update_route(renderer, geometry.encode())

  POSITIONS = [
    (32.71569271952601, -117.16384270868463, 0), (32.71569271952601, -117.16384270868463, 45),  # San Diego
    (32.71169219842112, -117.12514764382556, 45), (32.711701225668506, -117.12444222285058, 45),  # San Diego
  ]
  plt.figure()

  for i, pos in enumerate(POSITIONS):
    t = time.time()
    lib.map_renderer_update_position(renderer, *pos)
    wait_ready(lib, renderer)

    print(f"{pos} took {time.time() - t:.2f} s")

    plt.subplot(2, 2, i + 1)
    plt.imshow(get_image(lib, renderer))

  plt.show()
