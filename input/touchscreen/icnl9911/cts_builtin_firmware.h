static u8 icnl9911_driver_builtin_firmware[] = {
    // TODO:
};

const static struct cts_firmware cts_driver_builtin_firmwares[] = {
    {
        .name = "OEM-Project",      /* MUST set non-NULL */
#if defined(CONFIG_PROJ_T93_UCT)
        .hwid = CTS_DEV_HWID_ICNL9911C,
        .fwid = CTS_DEV_FWID_ICNL9911C,
#else  
        .hwid = CTS_DEV_HWID_ICNL9911,
        .fwid = CTS_DEV_FWID_ICNL9911,
#endif
        .data = icnl9911_driver_builtin_firmware,
        .size = ARRAY_SIZE(icnl9911_driver_builtin_firmware),
    },
};

