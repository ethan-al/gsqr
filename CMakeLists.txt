# GSQR config
build_lib(
  LIBNAME gsqr
  SOURCE_FILES
    model/gsqr-routing.cc
    model/gsqr-routing-protocol.cc
    model/gsqr-embedding.cc
    helper/gsqr-helper.cc
  HEADER_FILES
    model/gsqr-routing.h
    model/gsqr-routing-protocol.h
    model/gsqr-embedding.h
    helper/gsqr-helper.h
  LIBRARIES_TO_LINK
    libcore
    libnetwork
    libinternet
    libmobility
    libapplications
    libwifi
    libolsr
    libstats
)

# Comparison experiment for paper
build_lib_example(
  NAME gsqr-comparison
  SOURCE_FILES
    examples/gsqr-comparison.cc
  LIBRARIES_TO_LINK
    libgsqr
    libcore
    libnetwork
    libinternet
    libmobility
    libwifi
    libapplications
    libolsr
    libflow-monitor
)
