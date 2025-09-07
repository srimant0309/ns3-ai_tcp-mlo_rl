build_lib_example(
    NAME ns3ai_mlo_gym
    SOURCE_FILES
        mlo_environment.cc
    LIBRARIES_TO_LINK
        ${libai}                # AI module (ns3-ai)
        ${libcore}              # Core module
        ${libnetwork}           # Network module
        ${libinternet}          # Internet stack module
        ${libapplications}      # Applications module
        ${libpoint-to-point}    # Point-to-Point module (if needed)
        ${libspectrum}          # Spectrum module for wireless simulations
        ${libwifi}              # WiFi module for 802.11 simulations
        ${libpropagation}       # Propagation models
        ${libmobility}          # Mobility models
        ${libstats}             # Statistics module
        ${libtraffic-control}   # Traffic control module
        ${libinternet-apps}
)
