# MOD System Control

This is a tool that receives commands from MOD controller/HMI to perform systems actions (e.g. changing audio levels or enabling bluetooth discovery).

Additionally it also directly connects to [mod-host](https://github.com/mod-audio/mod-host) with a [custom IPC implementation](./src/sys_host_impl.h) for handling HMI Widgets. This mod-host direct connection allows to skip [mod-ui](https://github.com/mod-audio/mod-ui) as middle layer, which would be too slow for fast HMI Widget updates.

This tool is only used on MOD Duo X production units and MOD Dwarf.

## Building

The only dependency is "libserialport", optionally "systemd" for integration as a system service.

After installing the required dependencies, building can be done with:

```
make
```

This generates a `mod-system-control` binary.

## License

MOD Dwarf Controller is licensed under AGPLv3+, see [LICENSE](LICENSE) for more details.
