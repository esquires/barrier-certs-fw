# - Try to find  JSBSIM
# Once done, this will define
#
#  QUADPROGPP_FOUND - system has scicoslab 
#  QUADPROGPP_INCLUDE_DIRS - the scicoslab include directories
find_library(QUADPROGPP_LIBRARIES quadprog PATHS /usr/local/lib /usr/lib)
if (${QUADPROGPP_LIBRARIES} MATCHES "QUADPROGPP_LIBRARIES-NOTFOUND")
    set(QUADPROGPP_FOUND FALSE)
else() 
    set(QUADPROGPP_FOUND TRUE)
endif() 
