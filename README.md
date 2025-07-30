# PMW3610 driver for ZMK

This work is based on [badjeff's driver](https://github.com/badjeff/zmk-pmw3610-driver) with minor quality of life improvements (and 500 Hz). See badjeff's README for installation and usage.

### Differences 

#### KConfig

```
# logs surface quality, debug level, use only for debugging because it will prevent the sensor from going to sleep
CONFIG_PMW3610_SQUAL_LOG=n

# surface quality log interval, msec
CONFIG_PMW3610_SQUAL_LOG_INTERVAL=250

# ignore first N events after boot to avoid warping
CONFIG_PMW3610_IGNORE_FIRST_N=12

# ignore first events after rest too
CONFIG_PMW3610_IGNORE_AFTER_REST=y
```

#### Device Tree

```
  force-high-performance: boolean, replaces "force-awake-4ms-mode"
  xy-swap: boolean
  x-invert: boolean
  y-invert: boolean
  init-retry-count: int
  init-retry-interval: int
```
