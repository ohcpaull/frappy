from os import environ

# Simulated CF-17 cryostat.
#
# Same module tree as cfg/CF-17_cfg.py, but the two hardware IO endpoints are
# replaced by protocol-level simulators, so the real frappy_acns.lakeshore /
# frappy_acns.pfeiffer driver classes run unchanged with no instrument present.
#
# SECoP port defaults to 10767 (as the real node); override with
#   SIM_SECOP_PORT=10769 frappy-server sim_CF-17

Node('ls336.platypus.acns.sim',
     'simulated CF-17 cryostat (LakeShore 336 + Pfeiffer TPG)',
     interface='tcp://' + environ.get('SIM_SECOP_PORT', '10767'),
     secondary=['ws://10768'])

Mod('io',
    'frappy_acns.lakeshore_sim.Ls336Sim',
    'simulated LakeShore 336 controller',
    )
Mod('TPG_io',
    'frappy_acns.pfeiffer_sim.PfeifferTPGSim',
    'simulated Pfeiffer TPG vacuum gauge',
    )

Mod('T_HEx',
    'frappy_acns.lakeshore.TemperatureLoop',
    'Heat Exchanger Temperature',
    io='io',
    channel='A',  # the channel on the LakeShore for this module
    loop=1,  # the loop to be used
    value=Param(max=330),  # set the maximum expected T
    target=Param(max=325),  # set the maximum allowed target T
    heater_range=3,  #TODO: Check if heater range should change
    )
Mod('T_cold_probe',
    'frappy_acns.lakeshore.TemperatureLoop',
    'Sample Temperature for a cold stick',
    io='io',
    channel='B',  # the channel on the LakeShore for this module
    loop=2,  # the loop to be used
    value=Param(),  # set the maximum expected T
    target=Param(max=325),  # set the maximum allowed target T
    heater_range=3,  # 5 for model 350
    )

Mod('T_hot_probe',
    'frappy_acns.lakeshore.TemperatureLoop',
    'Sample Temperature for a hot stick',
    io='io',
    channel='C',  # the channel on the LakeShore for this module
    loop=2,  # the loop to be used
    value=Param(max=800),  # set the maximum expected T
    target=Param(max=750),  # set the maximum allowed target T
    heater_range=3,  # 5 for model 350
    )
Mod('T_cold_head',
    'frappy_acns.lakeshore.TemperatureSensor',
    'Temperature of the Cold Head',
    io='io',
    channel='D',  # the channel on the LakeShore for this module
    value=Param(max=470),  # set the maximum expected T
    )
Mod('Heater_HEx',
    'frappy_acns.lakeshore.Heater',
    'Heater Output for the Heat Exchanger',
    io='io',
    output=1,
    value=Param(max=100),
    #visibility='expert',
    )
Mod('Heater_cold_probe',
    'frappy_acns.lakeshore.Heater',
    'Heater output for the cold stick',
    io='io',
    output=2,
    value=Param(max=100)
    )
Mod('pressure',
    'frappy_acns.pfeiffer.PfeifferTPGPressureSensor',
    'Pressure sensor for the Pfeiffer TPG vacuum gauge',
    io='TPG_io',
    channel=1,
    )
