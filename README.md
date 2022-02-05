```shell
# Change directory to example
$ cd platform-sifive/examples/freedom-e-sdk_freertos-blinky

# Build project
$ pio run

# Upload firmware
$ pio run --target upload

# Build specific environment
$ pio run -e sifive-hifive1

# Upload firmware for the specific environment
$ pio run -e sifive-hifive1 --target upload

# Clean build files
$ pio run --target clean
```
